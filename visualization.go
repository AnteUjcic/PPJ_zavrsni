package main

// PREBAČENO - autor: Dino Gržinić

import (
	"fmt"
	"time"
)

// Ispis mreže
func printGrid(grid [][]*Node, start, goal *Node, path []*Node,
	openSet *PriorityQueue, closedSet map[*Node]bool, animationSpeed int) {
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
	// ANSI escape code
	fmt.Print("\033[H\033[2J")

	for y := range grid {
		for x := range grid[y] {
			node := grid[y][x]

			//IZMJENA -  autor novih znakova za ispis: Marin Rabađija
			switch {
			case node == start:
				fmt.Print(Blue + "A " + Reset)
			case node == goal:
				fmt.Print(Blue + "B " + Reset)
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
	// IZMJENA - autor: Marin Rabađija
	time.Sleep(time.Duration(animationSpeed) * time.Millisecond)
}

func reconstructPath(grid [][]*Node, start, goal *Node, openSet *PriorityQueue,
	closedSet map[*Node]bool, animationSpeed int) int {
	var path []*Node
	var current = goal
	var counter = 0

	// Rekonstruira putanju od cilja do početka
	for current != nil {
		path = append(path, current)
		current = current.Parent
		counter += 1

		// IZMJENA - autor: Marin Rabađija
		printGrid(grid, start, goal, path, openSet, closedSet, animationSpeed)
	}
	return counter - 2
}
