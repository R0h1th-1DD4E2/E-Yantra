if __name__ == '__main__':
    for i in range(int(input())):
        l = input().strip().lower()
        print('It is a palindrome') if l == l[::-1] else print('It is not a palindrome')
