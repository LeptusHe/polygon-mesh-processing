x_segment = 100
y_segment = 100

vertices = [[0, 0, 0] for y in range(0, y_segment + 1) for x in range(0, x_segment + 1)]
texcoords = [[0, 0] for y in range(0, y_segment + 1) for x in range(0, x_segment + 1)]
print(vertices)
print(texcoords)

for y in range(y_segment + 1):
    for x in range(x_segment + 1):
        index = y * (x_segment + 1) + x
        vertices[index] = [x, 0, y]
        texcoords[index] = [x / x_segment, y / y_segment]

print(vertices)
print(texcoords)


indices = []
for y in range(y_segment):
    for x in range(x_segment):
        v00 = y * (x_segment + 1) + x
        v01 = y * (x_segment + 1) + x + 1
        v10 = (y + 1) * (x_segment + 1) + x
        v11 = (y + 1) * (x_segment + 1) + x + 1

        indices.append([v00, v11, v01])
        indices.append([v00, v10, v11])



path = "../data/plane_test.obj"

with open(path, "w") as f:
    for vertex in vertices:
        f.write("v {} {} {}\n".format(vertex[0], vertex[1], vertex[2]))

    for texcoord in texcoords:
        f.write("vt {} {}\n".format(texcoord[0], texcoord[1]))

    for index in indices:
        f.write("f {0}/{0} {1}/{1} {2}/{2}\n".format(index[0] + 1, index[1] + 1, index[2] + 1))