import matplotlib.pyplot as plt
import numpy as np

amount = 100

lst = np.random.randint(0, 100, amount)
x = np.arange(0, amount)
n = len(lst)


def bubble(arr):
    plt.bar(x, arr)
    plt.title("Bubble sort")
    plt.pause(0.2)
    for i in range(n):
        for j in range(0, n - i - 1):
            plt.clf()
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
            plt.bar(x, arr)
            plt.title("Bubble Sort")
            plt.pause(0.2)
    plt.show()


def insertionSort(arr):
    plt.bar(x, lst)
    plt.title("Insertion Sort")
    plt.pause(0.001)
    for i in range(1, len(arr)):

        key = arr[i]
        j = i - 1
        plt.clf()
        while j >= 0 and key < arr[j]:
            arr[j + 1] = arr[j]
            j -= 1
        plt.bar(x, arr)
        plt.title("Insertion Sort")
        plt.pause(0.2)
        arr[j + 1] = key
    plt.show()


def selection_sort(A):
    plt.bar(x, lst)
    plt.title("Selection Sort")
    plt.pause(0.2)
    for i in range(len(A)):
        plt.clf()
        min_idx = i
        for j in range(i + 1, len(A)):
            if A[min_idx] > A[j]:
                min_idx = j
        A[i], A[min_idx] = A[min_idx], A[i]
        plt.bar(x, A)
        plt.title("Selection Sort")
        plt.pause(0.2)
    plt.show()


# bubble(lst)
# insertionSort(lst)
# selection_sort(lst)
