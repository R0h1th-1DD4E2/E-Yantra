if __name__ == '__main__':
    for _ in range(int(input())):
        l = [str(len(elem)) for elem in input()[1:].strip().split()]
        print(",".join(l))