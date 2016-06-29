/*
 *  linked_list.h
 *
 *  Created on: Sept 4, 2013
 *      Author: Walt Johnson
 */
#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#ifdef __cplusplus
extern "C" {
#endif


//_____ D E F I N I T I O N S ______________________________________________

typedef struct          // Place this structure at the start of any data structure to reference properly
{
    void                *prev;                 // Next object in linked list.  0 indicates this is the head.
    void                *nextCt;                 // Prev object in linked list.  0 indicates this is the tail.
}linked_list_node_t;

typedef struct
{
    linked_list_node_t  *head;                  // Head of linked list
    linked_list_node_t  *tail;                  // Tail of linked list
}linked_list_t;

//_____ P R O T O T Y P E S ________________________________________________

void linkedListClear( linked_list_t *ll );
void linkedListInsertAtHead( linked_list_t *ll, linked_list_node_t *newNode );
void linkedListInsertBefore( linked_list_t *ll, linked_list_node_t *node, linked_list_node_t *newNode );
void linkedListRemove( linked_list_t *ll, linked_list_node_t *node );


#ifdef __cplusplus
}
#endif

#endif // LINKED_LIST_H
