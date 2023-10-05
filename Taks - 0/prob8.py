'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

# Import reduce module
from functools import reduce


# Function to generate the A.P. series
def generate_AP(a1, d, n):
    AP_series = [a1 + (i-1)*d for i in range(1, n+1)]
    # Complete this function to return A.P. series
    return AP_series


# Main function
if __name__ == '__main__':
    # take the T (test_cases) input
    test_cases = int(input())
    # Write the code here to take the a1, d and n values
    for i in range(test_cases):
        # Once you have all 3 values, call the generate_AP function to find A.P. series and print it
        a1, d, n = list(map(int, input().split()))
        l = generate_AP(a1, d, n)
        AP_series = " ".join([str(i) for i in l])
        print(AP_series)
        # Using lambda and map functions, find squares of terms in AP series and print it
        sqr_AP_series = list(map(lambda x: x**2, l))
        print(" ".join([str(i) for i in sqr_AP_series]))
        # # Using lambda and reduce functions, find sum of squares of terms in AP series and print it
        sum_sqr_AP_series = reduce(lambda x, y: x+y, sqr_AP_series)
        print(sum_sqr_AP_series)
