package main

import (
	"container/heap" // Paket za implementaciju prioritetskog reda
	"fmt"
	"math"
	"time"
)

// Struktura čvora za A* algoritam
type Node struct {
	X, Y     int     // Koordinate čvora
	G, H, F  float64 // G: trošak od početka do trenutnog čvora, H: heuristika, F: ukupni trošak (G + H)
	Parent   *Node   // Pokazivač na roditeljski čvor za rekonstrukciju puta
	Index    int     // Indeks čvora u prioritetskom redu
	Walkable bool    // Označava je li čvor prohodan
}

// Prioritetski red (min heap) za čvorove
type PriorityQueue []*Node

// Implementacija sučelja za heap.Interface
func (pq PriorityQueue) Len() int { return len(pq) } // Vraća duljinu reda

// Uspoređuje F vrijednosti čvorova (manji F ima prednost)
func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].F < pq[j].F
}

// Zamjenjuje dva čvora u redu
func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

// Dodaje čvor u red
func (pq *PriorityQueue) Push(x interface{}) {
	n := x.(*Node)
	n.Index = len(*pq)
	*pq = append(*pq, n)
}

// Uklanja i vraća zadnji čvor iz reda
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	x := old[n-1]
	x.Index = -1
	*pq = old[0 : n-1]
	return x
}

// Ažurira podatke o čvoru i ponovno ga postavlja u red
func (pq *PriorityQueue) update(node *Node, g, h float64, parent *Node) {
	node.G = g
	node.H = h
	node.F = g + h
	node.Parent = parent
	heap.Fix(pq, node.Index)
}

// Heuristička funkcija (Manhattan udaljenost)
func heuristic(a, b *Node) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

func main() {
	// Definicija dimenzija mreže
	width, height := 10, 10

	// Inicijalizacija mreže čvorova
	grid := make([][]*Node, height)
	for y := 0; y < height; y++ {
		grid[y] = make([]*Node, width)
		for x := 0; x < width; x++ {
			grid[y][x] = &Node{
				X:        x,
				Y:        y,
				Walkable: true,
				Index:    -1,
			}
		}
	}

	// Definicija prepreka na mreži
	obstacles := []struct{ X, Y int }{
		{1, 0}, {2, 0}, {3, 0},
		{1, 1},
		{8, 2}, {8, 3}, {8, 4},
		{2, 4}, {2, 5}, {2, 6},
		{5, 5}, {5, 6}, {5, 7}, {5, 8},
		{6, 9},
	}

	// Označavanje prepreka kao neprohodnih čvorova
	for _, obs := range obstacles {
		grid[obs.Y][obs.X].Walkable = false
	}

	// Početni i krajnji čvor
	start := grid[0][0]
	goal := grid[3][9]

	// Pokretanje A* pretrage
	aStarSearch(grid, start, goal)
}

// Implementacija A* algoritma
func aStarSearch(grid [][]*Node, start, goal *Node) {
	openSet := &PriorityQueue{}
	heap.Init(openSet)

	start.G = 0
	start.H = heuristic(start, goal)
	start.F = start.G + start.H
	heap.Push(openSet, start)

	closedSet := make(map[*Node]bool)
	pathFound := false

	for openSet.Len() > 0 {
		current := heap.Pop(openSet).(*Node)
		closedSet[current] = true

		// Ako je cilj dosegnut, rekonstruiraj put
		if current == goal {
			pathFound = true
			break
		}

		// Obrada susjednih čvorova
		for _, neighbor := range getNeighbors(grid, current) {
			if !neighbor.Walkable || closedSet[neighbor] {
				continue
			}
			propG := current.G + 1
			// Ako susjed nije u openSetu ili je pronađen bolji put
			if neighbor.Index == -1 && !inOpenSet(neighbor) {
				neighbor.G = propG
				neighbor.H = heuristic(neighbor, goal)
				neighbor.F = neighbor.G + neighbor.H
				neighbor.Parent = current
				heap.Push(openSet, neighbor)
			} else if propG < neighbor.G {
				openSet.update(neighbor, propG, neighbor.H, current)
			}
		}

		printGrid(grid, start, goal, nil, openSet, closedSet)
	}

	// Ispis rezultata
	if pathFound {
		pathLength := reconstructPath(grid, start, goal, openSet, closedSet)
		fmt.Println("Duljina puta:", pathLength)
	} else {
		fmt.Println("Ne postoji put.")
	}
}

// Dohvaća susjedne čvorove
func getNeighbors(grid [][]*Node, node *Node) []*Node {
	directions := [][2]int{
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
	}
	var neighbors []*Node

	for _, d := range directions {
		nx, ny := node.X+d[0], node.Y+d[1]
		if ny >= 0 && ny < len(grid) && nx >= 0 && nx < len(grid[0]) {
			neighbors = append(neighbors, grid[ny][nx])
		}
	}
	return neighbors
}

// Provjerava je li čvor u openSetu
func inOpenSet(node *Node) bool {
	return node.Index != -1
}

func reconstructPath(grid [][]*Node, start, goal *Node, openSet *PriorityQueue, closedSet map[*Node]bool) int {
	var path []*Node
	var current = goal
	var counter = 0

	// Rekonstruira putanju od cilja do početka
	for current != nil {
		path = append(path, current)
		current = current.Parent
		counter += 1

		printGrid(grid, start, goal, path, openSet, closedSet)
	}
	return counter - 2
}

// Ispis mreže
func printGrid(grid [][]*Node, start, goal *Node, path []*Node, openSet *PriorityQueue, closedSet map[*Node]bool) {
	// Ispis vizualizacije
	pathMap := make(map[*Node]bool)
	for _, p := range path {
		pathMap[p] = true
	}

	openSetMap := make(map[*Node]bool)
	if openSet != nil {
		for _, n := range *openSet {
			openSetMap[n] = true
		}
	}

	Reset := "\033[0m"
	Red := "\033[31m"
	Green := "\033[32m"
	Blue := "\033[0;34m"
	Yellow := "\033[33m"
	fmt.Print("\033[H\033[2J")

	for y := range grid {
		for x := range grid[y] {
			node := grid[y][x]

			switch {
			case node == start:
				fmt.Print(Blue + "S " + Reset)
			case node == goal:
				fmt.Print(Blue + "G " + Reset)
			case pathMap[node]:
				fmt.Print(Green + "0 " + Reset) // Put
			case !node.Walkable:
				fmt.Print("# " + Reset) // Prepreka
			case openSet != nil && openSetMap[node]:
				fmt.Print(Yellow + "* " + Reset) // u setu
			case closedSet != nil && closedSet[node]:
				fmt.Print(Red + "* " + Reset) // zatvoren/posjećen
			default:
				fmt.Print(". ")
			}
		}
		fmt.Println()
	}
	fmt.Println()
	time.Sleep(150 * time.Millisecond)
}
