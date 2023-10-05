d = {}
ADD = 'ADD'
DELETE = 'DELETE'


def query(command: str) -> None:
    labels = command.split()
    amt = int(labels[2])
    if labels[0] == ADD:
        # print('ADDED Item {}'.format(labels[1]))
        if labels[1] not in d.keys():
            print('ADDED Item {}'.format(labels[1]))
            d[labels[1]] = int(amt)
        else:
            print('UPDATED Item {}'.format(labels[1]))
            d[labels[1]] += int(amt)
    elif labels[0] == DELETE:
        if labels[1] not in d.keys():
            print('Item {} does not exist'.format(labels[1]))
        else:
            if amt > int(d[labels[1]]):
                print('Item {} could not be DELETED'.format(labels[1]))
            else:
                print('DELETED Item {}'.format(labels[1]))
                d[labels[1]] -= amt
    else:
        assert "UNKNOWN OPERATION"


def main():
    for _ in range(int(input())):
        for _ in range(int(input())):
            a, b = input().split()
            b = int(b)
            d[a] = b
        for _ in range(int(input())):
            command = input()
            query(command)
        print('Total Items in Inventory:', sum(d.values()))


if __name__ == '__main__':
    main()
