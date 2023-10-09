if __name__ == '__main__':
    for _ in range(int(input())):
        n = int(input())
        l = list(map(int, input().strip().split()))
        print(" ".join([str(i) for i in l[::-1]]), end='')
        print(' ')
        l1, l2 = [], []
        for i in range(1, n):
            if i % 3 == 0:
                l1.append(str(l[i] + 3))
        print(" ".join(l1))
        for i in range(1, n):
            if i % 5 == 0:
                l2.append(str(l[i] - 7))
        print(" ".join(l2))
        print(sum(l[3:8]))
