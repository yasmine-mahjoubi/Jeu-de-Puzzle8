import tkinter as tk
from tkinter import messagebox
import random
import heapq
import time

# Définition de la grille du jeu
goal_state = [[0, 1, 2],
              [3, 4, 5],
              [6, 7, 8]]  # 0 représente la case vide

# Couleurs pour l'interface graphique
COLOR_BG = "white"
COLOR_EMPTY = "white"
COLOR_TILE = "purple"

# Fonction pour vérifier si l'état actuel correspond à l'état final
def is_goal(state):
    return state == goal_state

# Fonction pour générer un état initial aléatoire
def generate_random_state():
    state = [row[:] for row in goal_state] #copie de goal_state
    moves = [move_up, move_down, move_left, move_right]
    for _ in range(100): #'_' est utilisé comme une variable jetable, car sa valeur n'est pas utilisée dans le corps de la boucle.
        random.choice(moves)(state)
    return state

# Fonction pour déplacer la case vide vers le haut
def move_up(state):
    i, j = find_empty(state)
    if i > 0:
        state[i][j], state[i - 1][j] = state[i - 1][j], state[i][j] #permutation 

# Fonction pour déplacer la case vide vers le bas
def move_down(state):
    i, j = find_empty(state)
    if i < 2:
        state[i][j], state[i + 1][j] = state[i + 1][j], state[i][j]

# Fonction pour déplacer la case vide vers la gauche
def move_left(state):
    i, j = find_empty(state)
    if j > 0:
        state[i][j], state[i][j - 1] = state[i][j - 1], state[i][j]

# Fonction pour déplacer la case vide vers la droite
def move_right(state):
    i, j = find_empty(state)
    if j < 2:
        state[i][j], state[i][j + 1] = state[i][j + 1], state[i][j]

# Fonction pour trouver la position de la case vide
def find_empty(state):
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return i, j

# Fonction pour afficher l'état du jeu
def display_state(state):
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                label = tk.Label(root, text=" ", width=10, height=5, bg=COLOR_EMPTY)
            else:
                label = tk.Label(root, text=state[i][j], width=10, height=5, bg=COLOR_TILE)
                label.bind('<Button-1>', lambda e, i=i, j=j: on_tile_click(i, j)) #attache un gestionnaire d'événement à l'objet label. Ici, <Button-1> représente le clic gauche de la souris.
            label.grid(row=i, column=j, padx=5, pady=5) #positionne l'objet label dans la grille de l'interface Tkinter

# Fonction pour gérer le clic sur une tuile
def on_tile_click(i, j):
    global current_state, moves_count
    empty_i, empty_j = find_empty(current_state)
    if abs(empty_i - i) + abs(empty_j - j) == 1:
        current_state[empty_i][empty_j], current_state[i][j] = current_state[i][j], current_state[empty_i][empty_j]
        moves_count += 1
        display_state(current_state)
        if is_goal(current_state):
            messagebox.showinfo("Félicitations!", f"Vous avez gagné en {moves_count} mouvements!")

# Fonction pour mélanger les tuiles du puzzle
def shuffle():
    global current_state, moves_count
    current_state = generate_random_state()
    moves_count = 0
    display_state(current_state) #Met à jour l'affichage de l'état actuel.

# Algorithme A* pour résoudre le puzzle
def a_star_solve():
    path = a_star_search(current_state)
    if path: #si le chemin est trouvé 
        show_solution(path)

# Algorithme Hill Climbing pour résoudre le puzzle
def hill_climbing_solve():
    path = hill_climbing_search(current_state)
    if path: 
        show_solution(path)

# Affichage de la solution
def show_solution(path):
    for move in path:
        move(current_state)
        display_state(current_state)
        root.update() #Met à jour la fenêtre Tkinter
        time.sleep(0.5) #Attend 0,5 seconde entre chaque mouvement pour animer la solution
    if is_goal(current_state):
        messagebox.showinfo("Félicitations!", f"Résolu en {len(path)} mouvements!")

# Fonction d'évaluation heuristique pour A*
def heuristic(state):
    distance = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0:
                x, y = divmod(state[i][j], 3) #calcule sa position cible (x, y) et ajoute la distance de Manhattan à la distance heuristique
                distance += abs(x - i) + abs(y - j)
    return distance

# Fonction de recherche A*
def a_star_search(start):
    frontier = [] #ensemble ouvert
    heapq.heappush(frontier, (0 + heuristic(start), 0, start, [])) #Ajoute l'état initial avec son coût et chemin
    explored = set() #ensemble fermé 
    while frontier: #tant qu'il y a des états à explorer dans Ouvert
        _, cost, state, path = heapq.heappop(frontier) #Retire l'état avec le plus petit coût
        if is_goal(state):
            return path
        explored.add(tuple(map(tuple, state))) #Ajoute l'état actuel dans l'ensemble fermé
        for move in [move_up, move_down, move_left, move_right]: #Parcourt les quatre mouvements possibles
            new_state = [row[:] for row in state] #Crée une copie de l'état actuel
            move(new_state) #Applique le mouvement actuel à la copie de l'état pour obtenir un nouvel état
            if tuple(map(tuple, new_state)) not in explored:
                heapq.heappush(frontier, (cost + 1 + heuristic(new_state), cost + 1, new_state, path + [move]))
                #Ajoute le nouvel état dans ouvert avec un coût mis à jour, le nouvel état lui-même, et le chemin parcouru
                #jusqu'à cet état mis à jour avec le nouveau mouvement.
    return None

# Fonction de recherche Hill Climbing
def hill_climbing_search(start):
    current_state = [row[:] for row in start] #Crée une copie de l'état initial
    path = []
    while not is_goal(current_state):
        neighbors = [] #Initialise la liste des voisins
        for move in [move_up, move_down, move_left, move_right]:
            new_state = [row[:] for row in current_state] #copie de l'etat actuel
            move(new_state) #Applique le mouvement à la copie
            neighbors.append((heuristic(new_state), move, new_state)) #Ajoute le nouvel état à la liste des voisins avec
                                                                      #son heuristique et le mouvement
        neighbors.sort(key=lambda x: x[0]) #Trie les voisins par heuristique
        if heuristic(current_state) <= neighbors[0][0]: #Si aucun voisin n'est meilleur que l'état actuel, retourne le chemin
            return path
        current_state = neighbors[0][2] #Met à jour l'état actuel avec le meilleur voisin
        path.append(neighbors[0][1]) #Ajoute le mouvement au chemin
    return path

# Création de la fenêtre principale
root = tk.Tk()
root.title("8-Puzzle Game")
root.geometry("600x600")

# Boutons pour les algorithmes de résolution
a_star_button = tk.Button(root, text="A*", command=a_star_solve)
a_star_button.grid(row=4, column=0, columnspan=3)
hill_climbing_button = tk.Button(root, text="Hill Climbing", command=hill_climbing_solve)
hill_climbing_button.grid(row=5, column=0, columnspan=3)

# Bouton pour mélanger les tuiles
shuffle_button = tk.Button(root, text="Shuffle", command=shuffle)
shuffle_button.grid(row=6, column=0, columnspan=3)

# Initialisation de l'état actuel et des compteurs
current_state = generate_random_state()
moves_count = 0

# Affichage initial de l'état du jeu
display_state(current_state)

# Lancement de la boucle principale de l'interface graphique
root.mainloop()
