/*
 *  linked_list.c
 *
 *  Created on: Sept 4, 2013
 *      Author: Walt Johnson
 */
#include "linked_list.h"

// #define LL_VALIDATE_INPUT          // Uncomment to check that inputs are valid, not NULL.

void linkedListClear( linked_list_t *ll )
{
#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 )
        return;
#endif

    ll->head = 0;
    ll->tail = 0;
}


void linkedListInsertAtHead( linked_list_t *ll, linked_list_node_t *newNode )
{
#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || newNode==0 )
        return;
#endif

    if( ll->head )
    {   // Non-empty linked list.  
        ll->head->prev = newNode;
        newNode->nextCt = ll->head;
        newNode->prev = 0;
        ll->head = newNode;
    }
    else
    {   // Empty linked list.  Add to head.
        ll->head = newNode;
        ll->tail = newNode;
    }
}


void linkedListInsertBefore( linked_list_t *ll, linked_list_node_t *node, linked_list_node_t *newNode )
{
    linked_list_node_t *prev;

#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || node==0 || newNode==0 )
        return;
#endif

    // Reference previous node
    prev = (linked_list_node_t*)(node->prev);

    // Prev <-> newNode
    if( prev )
    {   // Node is NOT head of linked list (prev exists).
        prev->nextCt = newNode;
        newNode->prev = prev;
    }
    else
    {   // Node is first.  Insert at head.
        ll->head = newNode;
        newNode->prev = 0;
    }

    // newNode <-> next node
    node->prev = newNode;
    newNode->nextCt = node;
}


void linkedListRemove( linked_list_t *ll, linked_list_node_t *node )
{
    linked_list_node_t* prev;
    linked_list_node_t* next;

#ifdef LL_VALIDATE_INPUT
    // Validate Input
    if( ll==0 || node==0 )
        return;
#endif

    // Reference adjacent nodes
    prev = (linked_list_node_t*)(node->prev);
    next = (linked_list_node_t*)(node->nextCt);

    // Update Head and Tail pointers if needed
    if( ll->head == node )  
        ll->head = next;
    
    if( ll->tail == node )  
        ll->tail = prev;

    // Remove item from linked list, connect adjacent nodes
    if(prev)    
        prev->nextCt = next;

    if(next)    
        next->prev = prev;
}


