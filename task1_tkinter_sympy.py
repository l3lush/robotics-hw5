from tkinter import *
import math
# from sympy import Point, Polygon
from shapely.geometry import Point, Polygon
import heapq
import numpy as np

'''================= Your classes and methods ================='''

# These functions will help you to check collisions with obstacles
class Queue:
    "A container with a first-in-first-out (FIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Enqueue the 'item' into the queue"
        self.list.insert(0,item)

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the queue is empty"
        return len(self.list) == 0


class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

class PriorityQueueWithFunction(PriorityQueue):
    """
    Implements a priority queue with the same push/pop signature of the
    Queue and the Stack classes. This is designed for drop-in replacement for
    those two classes. The caller has to provide a priority function, which
    extracts each item's priority.
    """
    def  __init__(self, priorityFunction):
        "priorityFunction (item) -> priority"
        self.priorityFunction = priorityFunction      # store the priority function
        PriorityQueue.__init__(self)        # super-class initializer

    def push(self, item):
        "Adds an item to the queue with priority from the priority function"
        PriorityQueue.push(self, item, self.priorityFunction(item))


def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new+cx, y_new+cy))

    return new_points

def get_polygon_from_position(position):
    x,y,yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)] 
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    return Polygon(list(map(Point, new_points)))

def get_polygon_from_obstacle(obstacle) :
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])] 
    return Polygon(list(map(Point, points)))

def collides(position, obstacle) :
    return get_polygon_from_position(position).intersection(get_polygon_from_obstacle(obstacle))


class Stack:
    "A container with a last-in-first-out (LIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Push 'item' onto the stack"
        self.list.append(item)

    def pop(self):
        "Pop the most recently pushed item from the stack"
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the stack is empty"
        return len(self.list) == 0
        

class Window():
        
    '''================= Your Main Function ================='''
    
    def go(self, event):
    
        # Write your code here
                
        print("Start position:", self.get_start_position())
        print("Target position:", self.get_target_position()) 
        print("Obstacles:", self.get_obstacles())
        
        # Example of collision calculation
        
        number_of_collisions = 0
        for obstacle in self.get_obstacles() :
            if collides(self.get_start_position(), obstacle) :
                number_of_collisions += 1
        print("Start position collides with", number_of_collisions, "obstacles")

        self.explored = set()
        start_coord = self.get_start_position()
        start_coord = (round(start_coord[0], -1), round(start_coord[1], -1), round(start_coord[2] - math.pi / 2, 1))
        frontier = PriorityQueue()
        frontier.push((start_coord, None, 0), 0)
        cnt = 0

        try:
            A = math.tan(self.get_target_position()[2])
        except:
            A = 0
        B = 1
        C = self.get_target_position()[1] - A * self.get_target_position()[0]

        while not frontier.isEmpty():
            node = frontier.pop()
            self.explored.add(node[0])
            if self.is_goal_state(node[0]):
                break
            self.canvas.create_oval(node[0][0], node[0][1], node[0][0] + 10, node[0][1] + 10, fill='green')
            for child in self.generate_child(node[0]):
                child_x, child_y, child_theta = child
                if child not in self.explored and not self.touch_frame((child_x, child_y, child_theta)):
                    child_cost = self.distance(child_x, child_y, self.get_start_position()[0],
                                               self.get_start_position()[1])
                    child_cost += self.distance(child_x, child_y, self.get_target_position()[0],
                                                self.get_target_position()[1])
                    child_cost += 1 * abs(child_theta + math.pi / 2 - self.get_target_position()[2])
                    child_cost += 1 * (A * child_x + B * child_y + C) / (math.sqrt(A * A + B * B))
                    number_of_collisions = 0
                    for obstacle in self.get_obstacles():
                        if len(obstacle) == 8 and collides((child_x, child_y, child_theta), obstacle):
                            number_of_collisions += 1
                    if number_of_collisions == 0:
                        cnt += 1
                        print(cnt)
                        self.canvas.create_oval(child_x, child_y, child_x + 10, child_y + 10)
                        self.canvas.update()

                        self.explored.add((child_x, child_y, child_theta))
                        frontier.push(((child_x, child_y, child_theta), node, child_cost), child_cost)
        route = []
        while node[1]:
            route.append(node[0])
            node = node[1]

        for (x, y, _) in route:
            self.canvas.create_oval(x, y, x + 20, y + 20, fill='red')
        print('END')

    def touch_frame(self, state):
        x, y, _ = state
        if x - 50 > 0 and x + 50 < self.width and y - 100 > 0 and y + 100 < self.height:
            return False
        return True

    def generate_child(self, state):
        children = []
        x, y, theta = state
        phi = -round(math.pi / 4, 1)
        d = 200
        while phi < math.pi / 4:
            if math.isclose(phi, 0, abs_tol=1e-3):
                phi = 0.00001
            R = d / math.tan(phi)
            ICC = np.array([x - R * math.sin(theta), y + R * math.cos(theta)])
            linear_w = 20

            w = linear_w / R
            dt = 1
            first_matrix = np.array([[math.cos(w * dt), -math.sin(w * dt), 0],
                                     [math.sin(w * dt), math.cos(w * dt), 0],
                                     [0, 0, 1]])
            second_matrix = np.array([x - ICC[0], y - ICC[1], theta]).reshape(-1, 1)
            third_matrix = np.array([ICC[0], ICC[1], w * dt]).reshape(-1, 1)

            new_coords = (first_matrix @ second_matrix + third_matrix).flatten()
            new_coords = (round(new_coords[0], -1), round(new_coords[1], -1), round(new_coords[2], 1))
            if new_coords not in self.explored:
                children.append(new_coords)

            w = -linear_w / R
            first_matrix = np.array([[math.cos(w * dt), -math.sin(w * dt), 0],
                                     [math.sin(w * dt), math.cos(w * dt), 0],
                                     [0, 0, 1]])
            second_matrix = np.array([x - ICC[0], y - ICC[1], theta]).reshape(-1, 1)
            third_matrix = np.array([ICC[0], ICC[1], w * dt]).reshape(-1, 1)

            new_coords = (first_matrix @ second_matrix + third_matrix).flatten()
            new_coords = (round(new_coords[0], -1), round(new_coords[1], -1), round(new_coords[2], 1))
            if new_coords not in self.explored:
                children.append(new_coords)
            phi += 0.05
        return children

    def is_goal_state(self, state):
        target = self.get_target_position()
        target = (round(target[0], -1), round(target[1], -1), round(target[2] - math.pi / 2, 1))
        condition = math.isclose(state[0], target[0], rel_tol=0.01) and \
                    math.isclose(state[1], target[1], rel_tol=0.01) and \
                    (math.isclose(state[2], target[2], abs_tol=0.1) or math.isclose(-state[2], target[2], abs_tol=0.1))
        if condition:
            return True
        else:
            return False
        
    '''================= Interface Methods ================='''
    
    def get_obstacles(self) :
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2) :
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles
            
            
    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        yaw = self.get_yaw(2)
        return x,y,yaw
    
    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1 
        yaw = self.get_yaw(1)
        return x,y,yaw 
 

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y 
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw)
        else :
            return -math.acos(cos_yaw)
       
    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width  = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        # self.points = [0, 500, 500/2, 0, 500, 500]
    
if __name__ == "__main__":
    run = Window()
    run.run()
