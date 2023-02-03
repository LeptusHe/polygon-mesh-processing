import os


def generate_cmakelists():
    cmd = "xmake project -k cmakelists"
    os.system(cmd)


def set_mode(mode):
    cmd = "xmake config --mode={}".format(mode)
    os.system(cmd)


def build():
    cmd = "xmake -b"
    os.system(cmd)


def rebuild():
    cmd = "xmake -r"
    os.system(cmd)


if __name__ == "__main__":
    set_mode("debug")
    rebuild()
    generate_cmakelists()
