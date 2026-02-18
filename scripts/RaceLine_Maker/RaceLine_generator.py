'''
Author: Ramtin B. Meidani

This code aims to determine the most optimal path using A* algorithm for shortest path 
and the brushfire algorithm for the safest path. The aim is to combine both to get the shortest
and safest path.

The following program computes:

INPUT: .yaml and .pgm file
OUTPUT: .csv of the coordinates

Pillow documentation:
https://pillow.readthedocs.io/en/stable/reference/Image.html

yaml lib documentation:
https://pyyaml.org/wiki/PyYAMLDocumentation

A* example:
https://www.louispetit.be/teaching/a-demo

Brushfire:
voir dossier documentation

'''

import yaml
import os
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import sys
import csv
import cv2 

UNKNOWN  = -1
FREE     = 0
#OCCUPIED takes a percentage probability between 1 and 100. 100 means its for sure occupied
OCCUPIED = 100

#ouvrir fichier image
#ca dit pgm, mais ca peut ouvrir n'importe quel image
def pgm_opener(path):
    '''
    description: 
        ouvre un fichier image et le normalise
    params: 
        path: chemin vers le fichier image
    return: 
        img: image normalisé
    '''
    img = Image.open(path)

    #mettre en grayscale (important pour png)
    img = Image.open(path).convert("L")

    #normaliser de 0 a 1 au lieu de 0 à 255
    img = np.array(img) / 255.0
    return img


def yaml_opener(path):
    '''
    description: 
        ouvre un fichier yaml et sauvegarder les données dans une dictionnaire
    params: 
        path: chemin vers le fichier yaml
    return: 
        loaded_yaml: dictionnaire qui contient l'information du fichier
    '''
    #retourne dict du data
    with open(path, "r") as f:
        loaded_yaml = yaml.safe_load(f)
        
    return loaded_yaml

def saveCSV(data, filename):
    '''
    description: 
        sauvegarde les données dans un CSV
    params: 
        data: données à sauvegarder
        filename: nom du fichier
    return: 
        None
    '''
    file = filename + ".csv"

    with open(file, "w", newline='') as file:
        #les sauvegarder en colonnes. 
        #Pas evident, mais si on ouvre un csv sur vscode, on peut voir le delimiter
        write = csv.writer(file, delimiter=",")
        write.writerows(data)

def grid_generator(loaded_yaml, img):
    '''
    description: 
        Générer une carte métrique qui contient les dimensions réelles
    params: 
        loaded_yaml: dictionnaire avec information du fichier yaml
        img: l'image générée (pgm ou png)
    return: 
        occupancy_grid: carte métrique normalisé
    '''

    pgm      = loaded_yaml["image"]
    mode     = loaded_yaml["mode"]
    res      = loaded_yaml["resolution"]
    origin   = loaded_yaml["origin"]
    negate   = loaded_yaml["negate"]
    occ_thr = loaded_yaml["occupied_thresh"]
    free_thr = loaded_yaml["free_thresh"]

    #au cas ou le fichier est negate
    if negate:
        p = img 
    else:
        p = 1.0 - img

    height, width = img.shape
    occupancy_grid = np.full((height, width), UNKNOWN, dtype=np.int8)

    #notre cas, définir cases occupées et cases vides
    if mode == "trinary":
        occupancy_grid[p >= occ_thr]  = OCCUPIED
        occupancy_grid[p <= free_thr] = FREE

    elif mode == "scale":

        occupancy_grid = np.clip((1.0 - img) * 100, 0, 100).astype(np.int8)

    elif mode == "raw":
        occupancy_grid = (img * 255).astype(np.int16)
    else:
        raise ValueError(f"Unknown mode: {mode}")
    
    #occupancy_grid = np.flipud(occupancy_grid)

    return occupancy_grid


'''
def show_grid(occupancy_grid, raceline):
    vis = np.zeros_like(occupancy_grid, dtype=np.uint8)

    vis[occupancy_grid == -1]  = 127   
    vis[occupancy_grid == 0]   = 255   
    vis[occupancy_grid == 100] = 0     

    plt.figure()

    plt.imshow(occupancy_grid, cmap="inferno", origin="lower")
    plt.colorbar()
'''

def show_grid(occupancy_grid, raceline=None):
    '''
    description: 
        plotter la carte dessinée. Dans le cas nécessaire, avec le raceline
    params: 
        occupancy_grid: carte métrique normalisé
        raceline: ligne de trajectoire entre le début et la fin
    return: 
        None
    '''

    vis = np.zeros((occupancy_grid.shape[0], occupancy_grid.shape[1], 3), dtype=np.uint8)

    #remettre le fichier entre 0 et 255
    vis[occupancy_grid == -1] = [127, 127, 127]

    vis[occupancy_grid == 0]   = [240, 240, 255]  

    vis[occupancy_grid == 100] = [20, 20, 30]

    #dessiner les pixels pour la map
    if raceline:
        start = raceline[0]
        goal  = raceline[-1]
        vis[start[0], start[1]] = [0, 220, 0]
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            r, c = start[0] + dr, start[1] + dc
            if 0 <= r < vis.shape[0] and 0 <= c < vis.shape[1]:
                vis[r, c] = [0, 180, 0]

        vis[goal[0], goal[1]] = [220, 0, 0]
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            r, c = goal[0] + dr, goal[1] + dc
            if 0 <= r < vis.shape[0] and 0 <= c < vis.shape[1]:
                vis[r, c] = [180, 0, 0]

    #si c'est pas vide, dessine le raceline (polyline)
    if raceline and len(raceline) >= 2:
        
        points = np.array([(c, r) for r, c in raceline], dtype=np.int32)
        points = points.reshape((-1, 1, 2)) 

        cv2.polylines(
            img       = vis,
            pts       = [points],
            isClosed  = False,
            color     = (0, 220, 0),  
            thickness = 3,              
            lineType  = cv2.LINE_AA    
        )

    plt.figure(figsize=(12, 10))
    plt.imshow(vis, origin="lower") 
    plt.title("Occupancy Grid")

    #le if est peut etre useless ici
    #j'aurais pu endent tout ca avec le if qui dessine le polyline
    if raceline:
        plt.plot(raceline[0][1], raceline[0][0], 'go', markersize=12, label='Debut') 
        plt.plot(raceline[-1][1], raceline[-1][0], 'ro', markersize=12, label='Fin') 
        plt.legend(loc='upper right')

    plt.grid(False)
    plt.tight_layout()
    plt.show()

#code de Louis Petit, voir dossier documentation
def brushfire_algo(occupancyGrid, use8CellWindow=True):
    '''
    description: 
        Générer un heatmap des obstacles à éviter en utilisant l'algo brushfire
    params: 
        occupancy_grid: carte métrique normalisé
        use8CellWindow: Variable qui détermine si on veut utiliser connectivié par 8 ou par 4
    return: 
        occupancyGrid: Heatmap des obstacles
    '''

    nRows, nCols = occupancyGrid.shape
    a = 0

    # S'il y a des zéros, continue
    while np.any(occupancyGrid == 0):

        a += 1

        for iRow in range(nRows):
            for iCol in range(nCols):

                if occupancyGrid[iRow, iCol] == a:

                    # -------- 4-connected --------
                    if iRow > 0:
                        if occupancyGrid[iRow-1, iCol] == 0:
                            occupancyGrid[iRow-1, iCol] = a + 1

                    if iRow < nRows - 1:
                        if occupancyGrid[iRow+1, iCol] == 0:
                            occupancyGrid[iRow+1, iCol] = a + 1

                    if iCol > 0:
                        if occupancyGrid[iRow, iCol-1] == 0:
                            occupancyGrid[iRow, iCol-1] = a + 1

                    if iCol < nCols - 1:
                        if occupancyGrid[iRow, iCol+1] == 0:
                            occupancyGrid[iRow, iCol+1] = a + 1

                    # -------- 8-connected --------
                    if use8CellWindow:

                        if (iRow > 0) and (iCol > 0):
                            if occupancyGrid[iRow-1, iCol-1] == 0:
                                occupancyGrid[iRow-1, iCol-1] = a + 1

                        if (iRow > 0) and (iCol < nCols - 1):
                            if occupancyGrid[iRow-1, iCol+1] == 0:
                                occupancyGrid[iRow-1, iCol+1] = a + 1

                        if (iRow < nRows - 1) and (iCol > 0):
                            if occupancyGrid[iRow+1, iCol-1] == 0:
                                occupancyGrid[iRow+1, iCol-1] = a + 1

                        if (iRow < nRows - 1) and (iCol < nCols - 1):
                            if occupancyGrid[iRow+1, iCol+1] == 0:
                                occupancyGrid[iRow+1, iCol+1] = a + 1

    #un peu confusing comme nom de variable, mais c'est un heatmap
    return occupancyGrid

def A_star_algo(occupancy_grid, brushfire_weights, safety_weight ,start_pos, end_pos):
    '''
    description: 
        Trouver le chemin le plus court et le plus sécuritaire à l'aide de l'algo A* et brushfire
    params: 
        occupancy_grid: carte métrique normalisé
        brushfire_weights: heatmap des obstacles avec Brushfire
        safety_weight: Variable qui punit à quelle point on se rapproche des obstacles
        start_pos: position départ
        end_pos: position finale
    return: 
       path: chemin le plus court et le plus sécuritaire
    '''
    
    ROW = len(occupancy_grid)
    COL = len(occupancy_grid[0])

    #check si le début et fin sont dans la map (éviter des positions qui sortes de la map)
    if not (0 <= start_pos[0] < ROW and 0 <= start_pos[1] < COL):
        return None
    if not (0 <= end_pos[0] < ROW and 0 <= end_pos[1] < COL):
        return None
    
    #check si le début/fin est dans un obstacle
    if occupancy_grid[start_pos[0]][start_pos[1]] == OCCUPIED:
        print("start position is in an obstacle")
        return None
    if occupancy_grid[end_pos[0]][end_pos[1]] == OCCUPIED:
        print("destination is in an obstacle")
        return None

    #tracker les cases visités
    closed = [[False]*COL for _ in range(ROW)]

    #initialiser f cost et g cost A*
    f = [[float('inf')]*COL for _ in range(ROW)]
    g = [[float('inf')]*COL for _ in range(ROW)]
    
    #initialiser noeuds parent de A*
    parent = [[None]*COL for _ in range(ROW)]

    #h cost
    def h(r, c):
        return math.sqrt((r - end_pos[0])**2 + (c - end_pos[1])**2)

    #initialiser le noeud de départ
    sr, sc = start_pos
    #distance entre départ et lui-même est zéro
    g[sr][sc] = 0.0
    #donc f = g+h -> f = 0 + h
    f[sr][sc] = h(sr, sc)
    #le parent du noeud est lui-même
    parent[sr][sc] = (sr, sc)

    #ca ca gère les priorités des noeuds à visiter
    open_list = []
    heapq.heappush(open_list, (f[sr][sc], sr, sc))

    #tout les mouvements possibles
    directions = [
        (0, 1), (0, -1), (1, 0), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    while open_list:
        #noeud avec f cost le plus bas
        _, r, c = heapq.heappop(open_list)
        #indiquer que le noeud a été visité
        closed[r][c] = True

        for dr, dc in directions:
            #voisins
            nr, nc = r + dr, c + dc

            #skipper si les noeuds voisins sont dejas visités ou sont des obstacles
            if not (0 <= nr < ROW and 0 <= nc < COL):
                continue
            if occupancy_grid[nr][nc] == OCCUPIED:
                continue
            if closed[nr][nc]:
                continue

            #si le voisin est la destination
            if (nr, nc) == tuple(end_pos):
                parent[nr][nc] = (r, c)

                #creer le path
                path = []
                cur = (nr, nc)
                while True:
                    path.append(cur)
                    if parent[cur[0]][cur[1]] == cur:
                        break
                    cur = parent[cur[0]][cur[1]]
                path.reverse()
                return path

            #gestion des couts
            d = brushfire_weights[nr][nc]
            #éviter division par 0
            d_safe = np.clip(d, 1e-6, None)
            #formule de gestion de cout, ca peut changer pour d'autre choses, mais
            #ca c'est commun
            safety_cost = safety_weight / (d_safe**2)

            #cout de déplacement. 1 pour cardinale et sqrt(2) pour diagonale
            #move_cost = 1.0
            if dr != 0 and dc != 0:
                move_cost = math.sqrt(2)
            else:
                move_cost = 1.0

            #calcul de g cost
            g_new = g[r][c] + move_cost + safety_cost
            #calcul de f cost
            f_new = g_new + h(nr, nc)

            #update le voisin selon le nouveau path
            if f[nr][nc] > f_new:
                g[nr][nc] = g_new
                f[nr][nc] = f_new
                parent[nr][nc] = (r, c)
                heapq.heappush(open_list, (f_new, nr, nc))
    
    #aucun chemin trouvé
    return None 

def world_to_grid(world_pos, origin, resolution, height):
    '''
    description: 
        Convertir les coordonnées monde réelle en coordonnées carte métrique
    params: 
        world_pos: coordonnées (0,0) de la carte
        origin: coordonnées origines définit par le yaml
        resolution: resolution de la carte définit par le yaml
        height: hauteur de la carte
    return: 
        row: totale des rangées
        col: totale des colonnes
    '''
    x, y = world_pos[:2]
    ox, oy = origin[:2]

    #convertir metres à cases
    col = int((x - ox) / resolution)
    row = int((y - oy) / resolution)

    #flipper axe Y de la carte (Y augmente vers le haut en vrai vie, mais par le bas en image)
    row = height - 1 - row

    #changer l'ordre de colonne et rangée si nécessaire
    return (row, col)

if __name__ == "__main__":
    
    #chemin vers fichier yaml
    path_to_file = "Maps/"
    yaml_file = "1.yaml"
    full_yaml_path = os.path.join(path_to_file,yaml_file)

    #chemin vers fichier image. Ca peut etre pgm ou png, voir la doc PILLOW pour 
    #autres types de fichiers
    pgm_file = "1.pgm"
    full_pgm_path = os.path.join(path_to_file, pgm_file)

    #variable qui contient les informations yaml
    loaded_yaml = yaml_opener(full_yaml_path)

    #variable qui contient l'image
    loaded_img = pgm_opener(full_pgm_path)

    #carte métrique normalisée
    occupancy_grid = grid_generator(loaded_yaml, loaded_img)

    height, width = occupancy_grid.shape
    origin = loaded_yaml["origin"]
    res = loaded_yaml["resolution"]
    
    #origine de la carte
    start_world = [0.0, 0.0]
    
    #coordonnées destination [row, col]
    #les coordonnées ne sont pas en metres
    #si on les veut en metres, il faut utiliser world_to_grid
    end_pos = [500,900]

    #coordonnées de départ
    start_pos = world_to_grid(start_world, origin, res, height)
    print(start_pos)
    
    #initialiser carte métrique en binaire
    binary_grid = np.zeros_like(occupancy_grid, dtype=np.uint8)
    binary_grid[occupancy_grid == OCCUPIED] = 1

    brushfire_grid = np.zeros_like(binary_grid, dtype=np.int32)
    brushfire_grid[binary_grid == 1] = 1

    #heatmap brushfire pour les poids
    #ca nous aide à trouver le chemin le plus sécuritaire
    brushfire_grid = brushfire_algo(brushfire_grid)

    #à quel point on punit le mouvement proche des obstacles
    safety_weight = 50
    
    #générer le raceline
    raceline = A_star_algo(occupancy_grid, brushfire_grid, safety_weight, start_pos, end_pos)

    #le chemin ou on veut sauvegarder le fichier csv
    csv_repo = "saved/"
    #le nom du fichier
    csv_name = "map1"
    path_csv = os.path.join(csv_repo, csv_name)

    #si aucun chemin trouvé
    if raceline == None:
        print("No possible path")
    else:
        #sauvegarder le csv et plotter la carte
        saveCSV(raceline, path_csv)
        show_grid(occupancy_grid, raceline)
    

    