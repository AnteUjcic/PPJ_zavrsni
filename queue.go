package main

// PREBAČENO - autor: Dino Gržinić

import "container/heap"

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
