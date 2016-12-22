#ifndef ISLJPS_INCLUDE_JUMP_POINT_SEARCH_H_
#define ISLJPS_INCLUDE_JUMP_POINT_SEARCH_H_

#include <stddef.h>

#ifdef ISLJPS_STATIC
	#define ISLJPS_DEF static
#else
	#define ISLJPS_DEF extern
#endif

#ifndef ISLJPS_COST
	typedef double isljps_cost;
#else	
	typedef ISLJPS_COST isljps_cost;
#endif
	
#ifndef ISLJPS_MAX_COST
	#define ISLJPS_MAX_COST (1.0/0.0)
#endif

#ifndef ISLJPS_NODE_DATA
	#define ISLJPS_NODE_DATA
#endif

#ifndef ISLJPS_MAX_NEIGHBORS
	#define ISLJPS_MAX_NEIGHBORS 8
#endif

#ifndef ISLJPS_COORDS
	#ifndef ISLJPS_COORD
		typedef int isljps_coord;
	#else
		typedef ISLJPS_COORD isljps_coord;
	#endif
	#define ISLJPS_COORDS(x,y) isljps_coord x; isljps_coord y;
#endif

#define ISLJPS_HAS_NODE(graph,x,y) ((x) >= 0 && (y) >= 0 && (x) < ((graph)->width) && y < ((graph)->height))
#define ISLJPS_GET_NODE(graph,x,y) ((graph)->nodes + ((y) * (graph)->width) + (x))
#define ISLJPS_GET_NODE_OR_NULL(graph,x,y) (ISLJPS_HAS_NODE((graph),(x),(y)) ? ISLJPS_GET_NODE(graph,x,y) : NULL)

#ifndef ISLJPS_SQRT
	#include <math.h>
	#define ISLJPS_SQRT sqrt
#endif

#ifndef ISLJPS_MASK
	typedef int isljps_mask;
#else
	typedef ISLJPS_MASK isljps_mask;
#endif

#if !defined(ISLJPS_MALLOC)&&!defined(ISLJPS_REALLOC)&&!defined(ISLJPS_FREE)
	#include <stdlib.h>
	#define ISLJPS_MALLOC malloc
	#define ISLJPS_REALLOC realloc
	#define ISLJPS_FREE free
#elif !defined(ISLJPS_MALLOC)||!defined(ISLJPS_REALLOC)||!defined(ISLJPS_FREE)
	#error "You must to define ISLJPS_MALLOC, ISLJPS_REALLOC, ISLJPS_FREE to remove stdlib dependency"
#endif

#define ISLJPS_MASK_DIAG_0  1
#define ISLJPS_MASK_DIAG_1  2
#define ISLJPS_MASK_DIAG_2  3

typedef enum {
	ISLJPS_NODE_OPENED = 1,
	ISLJPS_NODE_CLOSED = 1 << 1,
} isljps_node_status;

#ifndef ISLJPS_NODE
typedef struct isljps_node isljps_node;

struct isljps_node {
	ISLJPS_NODE_DATA
	ISLJPS_COORDS(x,y)
	size_t index;
	isljps_mask mask;
	isljps_cost g;
	isljps_cost h;
	isljps_cost f;
	isljps_node_status status;
	isljps_node *parent;
};
#else
typedef ISLJPS_NODE isljps_node;
#endif

typedef struct {
	isljps_node *nodes;
	ISLJPS_COORDS(width,height)
} isljps_graph;

typedef int (*isljps_neighbors)( isljps_graph *, isljps_node *, isljps_mask, isljps_node ** );
typedef isljps_cost (*isljps_cost_fun)( isljps_graph *, isljps_node *, isljps_node *, isljps_mask mask );

typedef struct {
	isljps_neighbors get_neighbors;
	isljps_cost_fun eval_jump_cost;
	isljps_cost_fun eval_heuristic;
} isljps_properties;

#ifdef __cplusplus
extern "C" {
#endif

ISLJPS_DEF isljps_node **isljps_find_path( isljps_graph *graph, isljps_node *start, isljps_node *finish, isljps_mask mask, isljps_properties *properties );
ISLJPS_DEF int isljps_default_neighbors( isljps_graph *graph, isljps_node *node, isljps_mask mask, isljps_node *neighbors_out[] );
ISLJPS_DEF isljps_cost isljps_default_jump_cost( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask );
ISLJPS_DEF isljps_cost isljps_heuristic_euclidean( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask );
ISLJPS_DEF isljps_cost isljps_heuristic_manhattan( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask );
ISLJPS_DEF isljps_cost isljps_heuristic_chebyshev( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask );

#ifdef __cplusplus
}
#endif

static isljps_properties isljps_default_properties = {
	isljps_default_neighbors,
	isljps_default_jump_cost,
	isljps_heuristic_euclidean };

#endif // ISLJPS_INCLUDE_JUMP_POINT_SEARCH_H_

#ifdef ISL_JUMP_POINT_SEARCH_IMPLEMENTATION

#define ISLJPS_ABS(x) ((x)>0?(x):-(x)) 
#define ISLJPS_MAX(x,y) ((x)>(y)?(x):(y))
#define ISLJPS_SQRT2 (1.4142135623730951)

// Binary heap implementation for fast open list

typedef struct {
	isljps_node **nodes;
	size_t allocated;
	size_t length;
} isljps__heap;

static isljps__heap *isljps__heap_create( size_t n ) {
	isljps__heap *heap = ISLJPS_MALLOC( sizeof *heap );
	n = n <= 0 ? 1 : n;
	heap->nodes = ISLJPS_MALLOC( n * sizeof( *heap->nodes ));
	heap->allocated = n;
	heap->length = 0;
	return heap;
}

static void isljps__heap_destroy( isljps__heap *heap ) {
	ISLJPS_FREE( heap->nodes );
	ISLJPS_FREE( heap );
}

static int isljps__heap_grow( isljps__heap *heap, size_t newalloc ) {
	isljps_node **newnodes = ISLJPS_REALLOC( heap->nodes, sizeof( *heap->nodes ) * newalloc );
	if ( newnodes != NULL ) {
		heap->allocated = newalloc;
		heap->nodes = newnodes;
		return 0;
	} else {
		return 1;
	}
}

static void isljps__heap_swap( isljps__heap *heap, size_t index1, size_t index2 ) {
	isljps_node *tmp = heap->nodes[index1];
	heap->nodes[index1] = heap->nodes[index2];
	heap->nodes[index2] = tmp;
	heap->nodes[index1]->index = index1;
	heap->nodes[index2]->index = index2;
}

static size_t isljps__heap_siftup( isljps__heap *heap, size_t index ) {
	size_t parent = (index-1) >> 1;
	while ( index > 0 && heap->nodes[index]->f < heap->nodes[parent]->f ) {
		isljps__heap_swap( heap, index, parent );
		index = parent;
		parent = (index-1) >> 1;	
	}
	return index;
}

static void isljps__heap_siftdown_floyd( isljps__heap *heap, size_t index ) {
	size_t left = (index << 1) + 1;
	size_t right = left + 1;
	while ( left < heap->length ) {
		size_t higher = ( right < heap->length && heap->nodes[right]->f < heap->nodes[left]->f ) ? right : left;
		isljps__heap_swap( heap, index, higher );
		index = higher;
		left = (index << 1) + 1;
		right = left + 1;
	}
	isljps__heap_siftup( heap, index );
}

static void isljps__heap_siftdown( isljps__heap *heap, size_t index ) {
	size_t left = (index << 1) + 1;
	size_t right = left + 1;
	while ( left < heap->length ) {
		size_t higher = ( right < heap->length && heap->nodes[right]->f < heap->nodes[left]->f ) ? right : left;
		if ( heap->nodes[index]->f < heap->nodes[higher]->f ) 
			break;
		isljps__heap_swap( heap, index, higher );
		index = higher;
		left = (index << 1) + 1;
		right = left + 1;
	}
}

static void isljps__heap_enqueue( isljps__heap *heap, isljps_node *node ) {
	size_t index = heap->length;
	if ( heap->allocated <= heap->length ) {
		if ( isljps__heap_grow( heap, heap->allocated * 2 )) {
			return;
		}
	}
	heap->nodes[index] = node;
	heap->length++;
	index = isljps__heap_siftup( heap, index );
}

static isljps_node *isljps__heap_dequeue( isljps__heap *heap ) {
	if ( heap->length == 0 ) {
		return NULL;
	} else if ( heap->length == 1 ) {
		heap->length = 0;
		return heap->nodes[0];
	} else {
		isljps_node *node = heap->nodes[0];
		isljps__heap_swap( heap, 0, --heap->length );
		isljps__heap_siftdown_floyd( heap, 0 );
		return node;
	}
}

static void isljps__heap_update( isljps__heap *heap, isljps_node *node ) {
	isljps__heap_siftdown( heap, isljps__heap_siftup( heap, node->index ));
}

// End of binary heap implementation

static isljps_node **isljps__expand_path( isljps_graph *graph, isljps_node *start, isljps_node *finish ) {
	int allocated = (int) ISLJPS_SQRT((start->x-finish->x)*(start->x-finish->x) + (start->y-finish->y)*(start->y-finish->y));
	if ( allocated < 2 ) {
		return NULL;
	} else {
		isljps_node **path = ISLJPS_MALLOC( allocated );	
		isljps_node *node = finish;
		isljps_node *parent = node->parent;
		int length = 0;
		while ( parent != NULL ) {
			isljps_coord x0 = node->x;
			isljps_coord y0 = node->y;
			isljps_coord x1 = parent->x;
			isljps_coord y1 = parent->y;
			isljps_coord dx = ISLJPS_ABS( x0 - x1 );
			isljps_coord dy = ISLJPS_ABS( y0 - y1 );
			isljps_coord sx = x0 < x1 ? 1 : -1;
			isljps_coord sy = y0 < y1 ? 1 : -1;
			isljps_coord err = dx - dy;
			isljps_coord e2;

			if ( length+1 >= allocated ) {
				path = ISLJPS_REALLOC( path, allocated * 2 * sizeof( *path ));
				allocated = allocated * 2;
			}
			
			// Perform Bresenham's line algorithm to interpolate nodes between jump nodes
			while (x0 != x1 || y0 != y1) {
				path[length++] = ISLJPS_GET_NODE( graph, x0, y0 );

				e2 = 2 * err;
				if (e2 > -dy) {
					err = err - dy;
					x0 += sx;
				}
				if (e2 < dx) {
					err = err + dx;
					y0 += sy;
				}
			}

			node = parent;
			parent = node->parent;
		}
		path[length++] = node;
		return path;
	}
}

static isljps_node *isljps__jump( isljps_graph *graph, isljps_node *node, isljps_node *parent, isljps_node *finish, isljps_mask mask );

isljps_node **isljps_find_path( isljps_graph *graph, isljps_node *start, isljps_node *finish, isljps_mask mask, isljps_properties *properties ) {
	isljps__heap *openlist = isljps__heap_create( 8 );
	start->g = 0;
	start->f = 0;
	isljps__heap_enqueue( openlist, start );
	start->status |= ISLJPS_NODE_OPENED;
	properties = properties == NULL ? &isljps_default_properties : properties;

	while ( openlist->length > 0 ) {
		int i; 
		int count;
		isljps_node *neighbors[ISLJPS_MAX_NEIGHBORS];
	 	isljps_node *node = isljps__heap_dequeue( openlist );
		node->status |= ISLJPS_NODE_CLOSED;

		if ( node == finish ) {
			isljps__heap_destroy( openlist );
			return isljps__expand_path( graph, start, finish );
		}

		count = properties->get_neighbors( graph, node, mask, neighbors );
		for( i = 0; i < count; i++ ) {
			isljps_node *jump_node = isljps__jump( graph, neighbors[i], node, finish, mask );
			if ( jump_node != NULL ) {
				if ( !(jump_node->status & ISLJPS_NODE_CLOSED) ) {
					isljps_cost d = properties->eval_jump_cost( graph, jump_node, node, mask );
					isljps_cost ng = node->g + d;
					if ( !(jump_node->status & ISLJPS_NODE_OPENED) || ng < jump_node->g ) {
						jump_node->g = ng;
						jump_node->h = jump_node->h == 0 ? properties->eval_heuristic( graph, node, jump_node, mask ) : jump_node->h;
						jump_node->f = jump_node->g + jump_node->h;
						jump_node->parent = node;
						if ( !(jump_node->status & ISLJPS_NODE_OPENED )) {
							isljps__heap_enqueue( openlist, jump_node );
							jump_node->status |= ISLJPS_NODE_OPENED;
						} else {
							isljps__heap_update( openlist, jump_node );
						}
					}
				}
			}
		}
	}

	isljps__heap_destroy( openlist );

	return NULL;
}

static int isljps__is_node_walkable( isljps_graph *graph, isljps_coord x, isljps_coord y, isljps_mask mask ) {
	isljps_node *node = ISLJPS_GET_NODE_OR_NULL( graph, x, y );
	if ( node == NULL ) {
		return 0;
	} else {
		return (node->mask & mask) == node->mask;
	}
}

static isljps_node *isljps__jump( isljps_graph *graph, isljps_node *node, isljps_node *parent, isljps_node *finish, isljps_mask mask ) {
	if ( node == NULL || ((mask & node->mask) != node->mask )) {
		return NULL;
	} else if ( node == finish ) {
		return node;
	} else {
		isljps_coord x = node->x;
		isljps_coord y = node->y;
		isljps_coord dx = x - parent->x;
		isljps_coord dy = y - parent->x;
		if ( dx != 0 && dy != 0 ) {
			if ((isljps__is_node_walkable(graph, x + dx, y + dy, mask ) && !(isljps__is_node_walkable(graph, x + dx, y + dy, mask ))) ||
					(isljps__is_node_walkable(graph, x + dx, y - dy, mask ) && !(isljps__is_node_walkable(graph, x, y - dy, mask ))) ||
					(isljps__jump( graph, ISLJPS_GET_NODE_OR_NULL( graph, x + dx, y ), node, finish, mask ) != NULL) ||
					(isljps__jump( graph, ISLJPS_GET_NODE_OR_NULL( graph, x, y + dy ), node, finish, mask ) != NULL)) {
				return node;
			}	
		} else {
			if ( dx != 0 ) {
				if ((isljps__is_node_walkable(graph, x + dx, y + 1, mask ) && !isljps__is_node_walkable(graph, x, y + 1, mask )) ||
						(isljps__is_node_walkable(graph, x + dx, y - 1, mask ) && !isljps__is_node_walkable(graph, x, y - 1, mask ))) {
					return node;
				}
			} else {
				if ((isljps__is_node_walkable(graph, x + 1, y + dy, mask ) && !isljps__is_node_walkable(graph, x + 1, y, mask )) ||
						(isljps__is_node_walkable(graph, x - 1, y + dy, mask ) && !isljps__is_node_walkable(graph, x - 1, y, mask ))) {
					return node;
				}
			}
		}
		return isljps__jump( graph, ISLJPS_GET_NODE_OR_NULL( graph, x + dx, y + dy ), node, finish, mask );
	}
}

int isljps_default_neighbors( isljps_graph *graph, isljps_node *node, isljps_mask mask, isljps_node *neighbors_out[] ) {
	isljps_coord x = node->x;
	isljps_coord y = node->y;
	int count = 0;
	if ( node->parent != NULL ) {
		isljps_coord dx = x - node->parent->x;
		isljps_coord dy = y - node->parent->x;
		dx = dx / ISLJPS_MAX( ISLJPS_ABS( dx ), 1 ); 
		dy = dy / ISLJPS_MAX( ISLJPS_ABS( dy ), 1 );
		if ( dx != 0 && dy != 0 ) {
			if ( isljps__is_node_walkable(graph, x, y + dy, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x, y + dy );
			if ( isljps__is_node_walkable(graph, x + dx, y, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y );
			if ( isljps__is_node_walkable(graph, x + dx, y + dy, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y + dy );
			if ( !isljps__is_node_walkable(graph, x - dx, y, mask ) && ISLJPS_HAS_NODE( graph, x - dx, y + dy )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x - dx, y + dy );
			if ( !isljps__is_node_walkable(graph, x, y - dy, mask ) && ISLJPS_HAS_NODE( graph, x + dx, y - dy )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y - dy );
		} else {
			if ( dx == 0 ) {
				if ( isljps__is_node_walkable(graph, x, y + dy , mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x, y + dy );
				if ( !isljps__is_node_walkable(graph, x + 1, y , mask ) && ISLJPS_HAS_NODE( graph, x + 1, y + dy)) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + 1, y + dy );
				if ( !isljps__is_node_walkable(graph, x - 1, y , mask ) && ISLJPS_HAS_NODE( graph, x - 1, y + dy)) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x - 1, y + dy );
			} else {
				if ( isljps__is_node_walkable(graph, x + dx, y , mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y );
				if ( !isljps__is_node_walkable(graph, x, y + 1 , mask ) && ISLJPS_HAS_NODE( graph, x + dx, y + 1)) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y + 1 );
				if ( !isljps__is_node_walkable(graph, x, y - 1 , mask ) && ISLJPS_HAS_NODE( graph, x + dx, y - 1)) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + dx, y - 1 );
			}
		}
	} else {
		if ( isljps__is_node_walkable(graph, x + 1, y, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + 1, y );
		if ( isljps__is_node_walkable(graph, x - 1, y, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x - 1, y );
		if ( isljps__is_node_walkable(graph, x, y + 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x, y + 1 );
		if ( isljps__is_node_walkable(graph, x, y - 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x, y - 1 );
		if ( isljps__is_node_walkable(graph, x + 1, y - 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + 1, y - 1 );
		if ( isljps__is_node_walkable(graph, x - 1, y + 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x - 1, y + 1 );
		if ( isljps__is_node_walkable(graph, x - 1, y - 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + 1, y - 1 );
		if ( isljps__is_node_walkable(graph, x + 1, y + 1, mask )) neighbors_out[count++] = ISLJPS_GET_NODE( graph, x + 1, y + 1 );
	}
	return count;
}

isljps_cost isljps_default_jump_cost( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask ) {	
	isljps_coord dx = node1->x - node2->x;
	isljps_coord dy = node1->y - node2->y;
	dx = ISLJPS_ABS( dx );
	dy = ISLJPS_ABS( dy );
	return (dx<dy) ? ISLJPS_SQRT2 * dx + dy : ISLJPS_SQRT2 * dx + dy;
}

isljps_cost isljps_heuristic_manhattan( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask ) {
	isljps_coord dx = ( node1->x - node2->x );
	isljps_coord dy = ( node1->y - node2->y );
	dx = ISLJPS_ABS( dx );
	dy = ISLJPS_ABS( dy );
	return dx + dy;
}

isljps_cost isljps_heuristic_euclidean( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask ) {
	isljps_coord dx = ( node1->x - node2->x );
	isljps_coord dy = ( node1->y - node2->y );
	return ISLJPS_SQRT( dx*dx + dy*dy );
}

isljps_cost isljps_heuristic_chebyshev( isljps_graph *graph, isljps_node *node1, isljps_node *node2, isljps_mask mask ) {
	isljps_coord dx = ( node1->x - node2->x );
	isljps_coord dy = ( node1->y - node2->y );
	dx = ISLJPS_ABS( dx );
	dy = ISLJPS_ABS( dy );
	return ISLJPS_MAX( dx, dy );
}

#endif // ISL_JUMP_POINT_SEARCH_IMPLEMENTATION
