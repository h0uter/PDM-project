start = [5,5,9]
goal = [5,9,3]

spheres_r = []
spheres_r.append(4)

spheres_pos = []
spheres_pos.append([5, 5, 5])
n_spheres = 0

prisms = []
prisms.append([3, 5, 2, [90, 0, 0]])#prism width, length, height, rotation x, y, z in degrees

prisms_pos = []
prisms_pos.append([3, 6, 2])
n_prisms = 1

beams = []
beams.append([4, 10, 0.3, [0,0,0]]) #beams width, length, height, rotation x, y, z in degrees
beams.append([4, 10, 0.3, [0,0,0]])
beams.append([2, 7, 0.3, [0,0,0]])

beams.append([7, 0.3, 7.85, [0,0,0]])
beams.append([3, 0.3, 4.85, [0,0,0]])

beams.append([3, 0.3, 5, [0,0,0]])
beams.append([7, 0.3, 7.85, [0,0,0]])

beams.append([10, 0.1, 8, [0,0,0]])
beams.append([10, 0.1, 8, [0,0,0]])
beams.append([0.1, 10, 8, [0,0,0]])
beams.append([0.1, 10, 8, [0,0,0]])

beams_pos = []
beams_pos.append([2, 5, 8])
beams_pos.append([8, 5, 8])
beams_pos.append([5, 6.5, 8])

beams_pos.append([3.5, 2, 3.925])
beams_pos.append([8.5, 2, 5.425])

beams_pos.append([1.5, 4, 2.5])
beams_pos.append([6.5, 4, 3.925])

beams_pos.append([5, 0, 4])
beams_pos.append([5, 10, 4])
beams_pos.append([0, 5, 4])
beams_pos.append([10, 5, 4])

n_beams = 11

dronehitbox_r = 0.2
safety_margin = 0.3
