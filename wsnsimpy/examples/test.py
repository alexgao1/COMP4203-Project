import hashlib
import time
l = [1,2,3,4,5]
l2 = [6,7,8,9,10]
r = list(set(l) & set(l2))
print(r == [])
i = 12345
i2 = 67890
print(int(str(i) + str(i2)))
print(int(hashlib.md5("test".encode('utf-8')).hexdigest(), 16))
print(int(time.time()))
print(type(hashlib.md5("test".encode('utf-8')).hexdigest()))