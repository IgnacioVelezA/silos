import time

def script_A(duration = 10):

    for i in range(duration):
        print(i)
        time.sleep(1)

    open('script_A_finished.flag', 'w').close()

if __name__ == "__main__":
    script_A()
