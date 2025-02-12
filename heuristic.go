package main

// PREBAČENO - autor: Dino Gržinić

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
)

// DODAO Autor - Ante Ujčić
// HeuristicFunc predstavlja funkciju koja računa heuristiku između dva čvora.
type HeuristicFunc func(a, b *Node) float64

// DODAO Autor - Ante Ujčić
// Manhattan heuristika: zbraja apsolutne razlike koordinata
func manhattan(a, b *Node) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

// DODAO Autor - Ante Ujčić
// Euklidska heuristika: izračunava pravokutnu udaljenost
func euclidean(a, b *Node) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

// DODAO Autor - Ante Ujčić
// Dijagonalna heuristika: koristi kombinaciju ravnih i dijagonalnih pokreta
func diagonal(a, b *Node) float64 {
	dx := math.Abs(float64(a.X - b.X))
	dy := math.Abs(float64(a.Y - b.Y))
	D := 1.0
	D2 := math.Sqrt2
	return D*(dx+dy) + (D2-2*D)*math.Min(dx, dy)
}

// DODAO Autor - Ante Ujčić
// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
func getSelectedHeuristic() (HeuristicFunc, bool) {
	reader := bufio.NewReader(os.Stdin)
	for {
		fmt.Println("\nOdaberite heuristiku:")
		fmt.Println("1. Manhattan udaljenost")
		fmt.Println("2. Euklidska udaljenost")
		fmt.Println("3. Dijagonalna udaljenost")
		fmt.Print("Vaš odabir: ")

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)
		choice, err := strconv.Atoi(input)
		if err != nil || choice < 1 || choice > 3 {
			fmt.Println("Molimo unesite broj između 1 i 3")
			continue
		}
		switch choice {
		case 1:
			return manhattan, false
		case 2:
			return euclidean, true
		case 3:
			return diagonal, true
		}
	}
}

// DODANO - autor: Ante Ujčić
// IZMJENA - autor: Marin Rabađija (dodan indikator za implementaciju dijagonala)
func selectHeuristicFromValue(choice int) (HeuristicFunc, bool) {
	switch choice {
	case 1:
		return manhattan, false
	case 2:
		return euclidean, true
	case 3:
		return diagonal, true
	default:
		// Ako se dogodi neispravan unos, default je Manhattan.
		return manhattan, false
	}
}
