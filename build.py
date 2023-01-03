import os

def generate_cmakelists():
    cmd = "xmake project -k cmakelists"
    os.system(cmd)


def build():
    cmd = "xmake -b"
    os.system(cmd)


if __name__ == "__main__":
    build()
    generate_cmakelists()