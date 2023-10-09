def gen(i: int) -> int:
    return 3 if i == 0 else (i ** 2 if i % 2 != 0 else 2 * i)
if __name__ == '__main__':
    for _ in range(int(input())) :
        n = int(input())
        l = [str(gen(i)) for i in range(n)]
        print(" ".join(l))