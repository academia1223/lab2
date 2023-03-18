import random as rd
import time
from heapq import *
from tkinter import *
from tkinter.ttk import Combobox

import pygame as pg

width, height = 20, 20
block_size = 30
root = Tk()
clock = pg.time.Clock()



def close():
    global run
    run = False


def menu(timeend):
    run = True
    while run:
        image = PhotoImage(file='map.png')
        lbl = Label(frame_image)
        lbl.image = image
        lbl['image'] = lbl.image
        lbl.place(x=0, y=0)
        label = Label(frame_but, text=f"Время выполнения: {timeend}")
        label.place(relx=0.2, rely=0.8)
        root.update()
        time.sleep(0.01)

    root.destroy()


def rect(x, y):
    return x * block_size + 1, y * block_size + 1, block_size - 2, block_size - 2


def get_line(x, y):
    return [x * block_size + 15, y * block_size + 15]


def get_circle(x, y):
    return (x * block_size + block_size // 2, y * block_size + block_size // 2), block_size // 4


def map_generation():
    global start
    global goal
    global grid
    global wall
    pg.init()
    map = pg.display.set_mode([width * block_size, height * block_size])
    grid = [[10000 if rd.random() < 0.2 else 2 if rd.random() > 0.5 else 1
             for column in range(width)] for row in range(height)]
    wall = []
    for i in range(len(grid) - 1):
        for j in range(len(grid[i]) - 1):
            if grid[i][j] == 10000:
                wall.append((j, i))
    while True:
        xST, yST = rd.randint(0, width - 1), rd.randint(0, height - 1)
        xGL, yGL = rd.randint(0, width - 1), rd.randint(0, height - 1)
        start = (xST, yST)
        goal = (xGL, yGL)
        if grid[xST][yST] == 10000:
            continue
        elif grid[xGL][yGL] == 10000 or goal == start or heuristic(start, goal) < 5:
            continue
        else:
            break
    map.fill(pg.Color('white'))
    # draw grid
    [[pg.draw.rect(map, pg.Color('black'), rect(x, y), border_radius=block_size // 10)
      for x, column in enumerate(row) if column == 10000] for y, row in enumerate(grid)]
    [[pg.draw.rect(map, pg.Color('blue'), rect(x, y), border_radius=block_size // 10)
      for x, column in enumerate(row) if column == 2] for y, row in enumerate(grid)]
    pg.draw.rect(map, pg.Color('red'), rect(*start), border_radius=block_size // 10)
    pg.draw.rect(map, pg.Color('green'), rect(*goal), border_radius=block_size // 10)
    pg.display.flip()
    pg.image.save(map, 'map.png')
    pg.quit()
    timeend=0
    menu(timeend)


def get_neighbour(x, y):
    check = lambda x, y: True if 0 <= x < width and 0 <= y < height else False
    ways = [-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]
    # ways = [-1, 0], [0, 1], [1, 0], [0, -1]
    return [(grid[y + dy][x + dx], (x + dx, y + dy)) for dx, dy in ways if check(x + dx, y + dy)]


def heuristic(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def deikstra():
    pg.init()
    map = pg.display.set_mode([width * block_size, height * block_size])
    graph = {}
    for y, row in enumerate(grid):
        for x, column in enumerate(row):
            if column:
                graph[(x, y)] = graph.get((x, y), []) + get_neighbour(x, y)
    queue = []
    cost_visited = {start: 0}
    heappush(queue, (0, start))
    visited = {start: None}
    cur_node = start
    timestart = time.time()
    while True:
        # fill screen
        map.fill(pg.Color('white'))
        # draw grid
        [[pg.draw.rect(map, pg.Color('black'), rect(x, y), border_radius=block_size // 10)
          for x, column in enumerate(row) if column == 10000] for y, row in enumerate(grid)]
        [[pg.draw.rect(map, pg.Color('blue'), rect(x, y), border_radius=block_size // 10)
          for x, column in enumerate(row) if column == 2] for y, row in enumerate(grid)]
        pg.draw.rect(map, pg.Color('red'), rect(*start), border_radius=block_size // 10)
        pg.draw.rect(map, pg.Color('green'), rect(*goal), border_radius=block_size // 10)
        if queue:
            cur_cost, cur_node = heappop(queue)
            if cur_node == goal:
                queue = []
                continue
            next_nodes = graph[cur_node]
            for next_node in next_nodes:
                neigh_cost, neigh_node = next_node
                new_cost = cost_visited[cur_node] + neigh_cost
                if neigh_node not in cost_visited or new_cost < cost_visited[neigh_node]:
                    heappush(queue, (new_cost, neigh_node))
                    cost_visited[neigh_node] = new_cost
                    visited[neigh_node] = cur_node
        path_head, path_segment = cur_node, cur_node
        while path_segment and path_segment in visited:
            pg.draw.circle(map, pg.Color('darkorange'), *get_circle(*path_segment))
            path_segment = visited[path_segment]
        pg.draw.circle(map, pg.Color('green'), *get_circle(*start))
        pg.draw.circle(map, pg.Color('magenta'), *get_circle(*path_head))
        pg.display.flip()
        if queue == [] and cur_node == goal:
            timeend = time.time()-timestart
            break
    pg.image.save(map, 'map.png')
    pg.quit()
    menu(round(timeend,3))


def astar():
    pg.init()
    map = pg.display.set_mode([width * block_size, height * block_size])
    graph = {}
    for y, row in enumerate(grid):
        for x, column in enumerate(row):
            if column:
                graph[(x, y)] = graph.get((x, y), []) + get_neighbour(x, y)
    queue = []
    cost_visited = {start: 0}
    heappush(queue, (0, start))
    visited = {start: None}
    cur_node = start
    timestart = time.time()
    while True:
        # fill screen
        map.fill(pg.Color('white'))
        # draw grid
        [[pg.draw.rect(map, pg.Color('black'), rect(x, y), border_radius=block_size // 10)
          for x, column in enumerate(row) if column == 10000] for y, row in enumerate(grid)]
        [[pg.draw.rect(map, pg.Color('blue'), rect(x, y), border_radius=block_size // 10)
          for x, column in enumerate(row) if column == 2] for y, row in enumerate(grid)]
        pg.draw.rect(map, pg.Color('red'), rect(*start), border_radius=block_size // 10)
        pg.draw.rect(map, pg.Color('green'), rect(*goal), border_radius=block_size // 10)
        if queue:
            cur_cost, cur_node = heappop(queue)
            if cur_node == goal:
                queue = []
                continue
            next_nodes = graph[cur_node]
            for next_node in next_nodes:
                neigh_cost, neigh_node = next_node
                new_cost = cost_visited[cur_node] + neigh_cost
                if neigh_node not in cost_visited or new_cost < cost_visited[neigh_node]:
                    priority = new_cost + heuristic(neigh_node, goal)
                    heappush(queue, (priority, neigh_node))
                    cost_visited[neigh_node] = new_cost
                    visited[neigh_node] = cur_node

        path_head, path_segment = cur_node, cur_node
        while path_segment and path_segment in visited:
            pg.draw.circle(map, pg.Color('darkorange'), *get_circle(*path_segment))
            path_segment = visited[path_segment]
        pg.draw.circle(map, pg.Color('green'), *get_circle(*start))
        pg.draw.circle(map, pg.Color('magenta'), *get_circle(*path_head))
        pg.display.flip()
        if queue == [] and cur_node == goal:
            timeend = time.time()-timestart
            break
    pg.image.save(map, 'map.png')
    pg.quit()
    menu(round(timeend,3))


def move(a, tgoal, goalpos):
    check = lambda x, y: True if 0 <= x < width and 0 <= y < height else False
    temp = a
    path_tp = {temp: None}
    while temp != tgoal:
        dx = tgoal[0] - a[0]
        dy = tgoal[1] - a[1]
        if dx == 0 and dy > 0:
            xn, yn = temp[0], temp[1] + 1
        elif dx > 0 and dy > 0:
            xn, yn = temp[0] + 1, temp[1] + 1
        elif dx > 0 and dy == 0:
            xn, yn = temp[0] + 1, temp[1]
        elif dy < 0 < dx:
            xn, yn = temp[0] + 1, temp[1] - 1
        elif dx == 0 and dy < 0:
            xn, yn = temp[0], temp[1] - 1
        elif dx < 0 and dy < 0:
            xn, yn = temp[0] - 1, temp[1] - 1
        elif dx < 0 and dy == 0:
            xn, yn = temp[0] - 1, temp[1]
        else:
            xn, yn = temp[0] - 1, temp[1] + 1

        if check(xn, yn):
            if (xn, yn) not in wall:
                if grid[yn][xn] != 10000:
                    path_tp[(xn, yn)] = temp
                    temp = (xn, yn)
                    if temp == goalpos:
                        break
                else:
                    break
            else:
                break
        else:
            break
    return temp, path_tp


def rrt(startpos, goalpos, grid):
    pg.init()
    map = pg.display.set_mode([width * block_size, height * block_size])
    counter = 0
    map.fill(pg.Color('white'))
    [[pg.draw.rect(map, pg.Color('black'), rect(x, y), border_radius=block_size // 10)
      for x, column in enumerate(row) if column == 10000] for y, row in enumerate(grid)]
    [[pg.draw.rect(map, pg.Color('blue'), rect(x, y), border_radius=block_size // 10)
      for x, column in enumerate(row) if column == 2] for y, row in enumerate(grid)]
    pg.draw.rect(map, pg.Color('red'), rect(*start), border_radius=block_size // 10)
    pg.draw.rect(map, pg.Color('green'), rect(*goal), border_radius=block_size // 10)
    nodes = [startpos]
    graph = {startpos: None}
    cur_pos = startpos
    node = startpos
    near_goal = [goalpos] + get_neighbour(*goalpos)
    timestart = time.time()
    while cur_pos != goalpos:
        counter += 1
        xtp, ytp = rd.randint(0, width - 1), rd.randint(0, height - 1)
        tempGoal = (xtp, ytp)
        d = 1000
        for i in nodes:
            dtemp = heuristic(i, tempGoal)
            if dtemp < d:
                d = dtemp
                node = i
        cur_pos = node
        node, path = move(node, tempGoal, goalpos)
        if counter == 1000:
            timeend = time.time()-timestart
            break
        if node not in nodes and cur_pos != node:
            nodes.append(node)
            graph[node] = cur_pos

            path_head, path_segment = node, node
            while path_segment and path[path_segment] is not None:
                temp = path[path_segment]
                pg.draw.line(map, pg.Color('red'), get_line(*temp), get_line(*path_segment), 2)
                path_segment = path[path_segment]
        if node in near_goal:
            if node == goalpos:
                timeend = time.time()-timestart
                break
            continue
    path_head, path_segment = node, node
    while path_segment and graph[path_segment] is not None:
        temp = graph[path_segment]
        pg.draw.line(map, pg.Color('darkorange'), get_line(*temp), get_line(*path_segment), 5)
        path_segment = graph[path_segment]
    pg.draw.circle(map, pg.Color('green'), *get_circle(*start))
    pg.draw.circle(map, pg.Color('magenta'), *get_circle(*path_head))
    pg.display.flip()
    pg.image.save(map, "map.png")
    pg.quit()
    menu(round(timeend,3))


def starter():
    get = combo.get()
    if get == 'Деикстра':
        deikstra()
    elif get == 'A*':
        astar()
    else:
        rrt(start, goal, grid)


root.title('Pathfinding')
root.geometry('1000x600')
root.resizable(width=False, height=False)
root.wm_attributes("-topmost", 1)

pg.init()
global map
map = pg.display.set_mode([width * block_size, height * block_size])
map.fill(pg.Color('white'))
pg.image.save(map, 'map.png')
pg.quit()
root.protocol("WN_DELETE_WINDOW", close)
# фрейм для кнопок
frame_but = Frame(root, bg='plum1')
frame_but.place(relwidth=0.4, relheight=1, relx=0.6)

# фрейм для картинки
frame_image = Frame(root, bg='red')
frame_image.place(relwidth=0.6, relheight=1)

# картинка
python_logo = PhotoImage(file="map.png")
label = Label(frame_image, image=python_logo)
label.pack()

btn_map = Button(frame_but, text='Сгенерировать карту', command=map_generation, width=30, height=3)
btn_map.place(rely=0.1, relx=0.2)

combo = Combobox(frame_but, width=35)
combo['values'] = ("Деикстра", 'A*', 'RRT')
combo.current(0)
combo.place(rely=0.3, relx=0.2)

btn_start = Button(frame_but, text='Старт', command=starter, width=30, height=3)
btn_start.place(rely=0.5, relx=0.2)

image = PhotoImage(file='map.png')
lbl = Label(frame_image)
lbl.image = image
lbl['image'] = lbl.image
lbl.place(x=0, y=0)
timeend=0
menu(timeend)
