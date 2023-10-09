for i in range(int(input())):
    n = int(input())
    l = []
    for j in range(n):
        a, b = input().split()
        b = float(b)
        l.append([a, b])
    new_l = sorted(l, key=lambda x: x[1], reverse=True)
    names = []
    max_value = new_l[0][1]
    for name, value in new_l:
        if value == max_value:
            names.append(name)
    names.sort()
    print("\n".join(names))
