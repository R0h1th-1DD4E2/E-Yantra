if __name__ == '__main__':
    for _ in range(int(input())):
        n=int(input())
        for i in range(n, 0, -1):
            for j in range(1, i+1):
                print('#', end='') if j % 5 == 0 else print('*', end='')
            print()

