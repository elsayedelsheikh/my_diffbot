"""SRP-6a (RFC 5054 math) and the session keys derived from it. Pure logic.

The phone proves it knows the robot PIN without sending it, and both sides get a
shared key K that an eavesdropper or a fake robot can't compute: each guess at the
PIN costs one live attempt against the robot (rate-limited by protocol.Lockout).
Mirrored byte for byte by merlin_android .../Srp.kt.
"""

import hashlib
import hmac
import secrets

# RFC 5054 Appendix A, 2048-bit group: A/B are 256 bytes, so every handshake message
# fits one 512-byte GATT value even base64-encoded.
N_2048 = int(
    'AC6BDB41324A9A9BF166DE5E1389582FAF72B6651987EE07FC3192943DB56050A37329CBB4A099ED'
    '8193E0757767A13DD52312AB4B03310DCD7F48A9DA04FD50E8083969EDB767B0CF6095179A163AB3'
    '661A05FBD5FAAAE82918A9962F0B93B855F97993EC975EEAA80D740ADBF4FF747359D041D5C33EA7'
    '1D281E446B14773BCA97B43A23FB801676BD207A436C6481F1D2B9078717461A5B9D32E688F87748'
    '544523B524B0D57D5EA77A2775D2ECFA032CFBDBF52FB37861602790'
    '04E57AE6AF874E7303CE53299CCC041C7BC308D82A5698F3A8D0C38271AE35F8E9DBFBB6'
    '94B5C803D89F7AE435DE236D525F54759B65E372FCD68EF20FA7111F9E4AFF73', 16)
G = 2
IDENTITY = b'merlin'  # fixed: renaming the robot must not change the PIN's verifier


def pad(n, N):
    return n.to_bytes((N.bit_length() + 7) // 8, 'big')


def H(hash_name, *parts):
    h = hashlib.new(hash_name)
    for p in parts:
        h.update(p)
    return h.digest()


def to_int(b):
    return int.from_bytes(b, 'big')


def multiplier(hash_name, N, g):
    return to_int(H(hash_name, pad(N, N), pad(g, N)))  # k = H(N | PAD(g))


def private_key(hash_name, salt, identity, password):
    return to_int(H(hash_name, salt, H(hash_name, identity, b':', password)))  # x


def verifier(hash_name, N, g, salt, identity, password):
    return pow(g, private_key(hash_name, salt, identity, password), N)


def scramble(hash_name, N, A, B):
    return to_int(H(hash_name, pad(A, N), pad(B, N)))  # u = H(PAD(A) | PAD(B))


def server_public(hash_name, N, g, v, b):
    return (multiplier(hash_name, N, g) * v + pow(g, b, N)) % N  # B = k*v + g^b


def server_secret(N, A, v, u, b):
    return pow(A * pow(v, u, N) % N, b, N)  # S = (A * v^u)^b


def client_secret(hash_name, N, g, x, a, u, B):
    k = multiplier(hash_name, N, g)
    return pow((B - k * pow(g, x, N)) % N, a + u * x, N)  # S = (B - k*g^x)^(a + u*x)


def proofs(hash_name, N, A, B, S):
    """(K, M1, M2): session key, client proof, server proof."""
    K = H(hash_name, pad(S, N))
    m1 = H(hash_name, pad(A, N), pad(B, N), K)
    return K, m1, H(hash_name, pad(A, N), m1, K)


def hkdf(key, info, length=32, salt=b'merlin-ble-2'):
    """RFC 5869 HKDF-SHA256."""
    prk = hmac.new(salt, key, 'sha256').digest()
    out, block = b'', b''
    for i in range(1, -(-length // 32) + 1):
        block = hmac.new(prk, block + info + bytes([i]), 'sha256').digest()
        out += block
    return out[:length]


def session_keys(K):
    """(phone->robot key, robot->phone key) for AES-256-GCM."""
    return hkdf(K, b'phone->robot'), hkdf(K, b'robot->phone')


class Server:
    """The robot's half of one handshake, SHA-256 over the 2048-bit group."""

    HASH = 'sha256'

    def __init__(self, pin, salt=None, b=None):
        self.salt = salt or secrets.token_bytes(16)
        self._v = verifier(self.HASH, N_2048, G, self.salt, IDENTITY, pin.encode())
        self._b = b or secrets.randbits(256)
        self.B = server_public(self.HASH, N_2048, G, self._v, self._b)

    def verify(self, A, m1):
        """K and M2 if the phone's proof M1 checks out, else None."""
        if A % N_2048 == 0:
            return None  # A = 0 (mod N) would make S predictable
        u = scramble(self.HASH, N_2048, A, self.B)
        K, expected, m2 = proofs(self.HASH, N_2048, A, self.B,
                                 server_secret(N_2048, A, self._v, u, self._b))
        return (K, m2) if hmac.compare_digest(m1, expected) else None
