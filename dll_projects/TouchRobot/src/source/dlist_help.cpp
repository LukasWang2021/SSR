#include <stdio.h>
#include <stdlib.h>
#include "dlist_help.h"

dlist_node_t *circle_dlist_init(void)
{
	dlist_node_t *phead = (dlist_node_t * )malloc(sizeof(dlist_node_t));
	phead->pdata = NULL;
	phead->pnext = phead;
	phead->pprev = phead;
	return phead;
}

dlist_node_t *circle_dlist_tail_insert(dlist_node_t *phead, char *pdata)
{
    dlist_node_t *pnode = (dlist_node_t*)malloc(sizeof(dlist_node_t));
    pnode->pdata = pdata;

	pnode->pprev = phead->pprev;
    pnode->pnext = phead;

	phead->pprev->pnext = pnode;
	phead->pprev = pnode;

	return phead;
}

int circle_dlist_get_nums(dlist_node_t *phead)
{
	int cnt = 0;
	dlist_node_t *pnode = phead;
	
	while(pnode->pnext != phead)
	{
		pnode = pnode->pnext;
		cnt++;
	}
	return cnt;
}


char *circle_dlist_head_eat(dlist_node_t* phead)
{
	char *pdata = NULL;
    dlist_node_t *pnode = phead->pnext;
    if(pnode == phead)
		return NULL;

    pdata = pnode->pdata;
    phead->pnext = pnode->pnext;
    pnode->pnext->pprev = phead;

    free(pnode);
	return pdata;
}

char *circle_dlist_tail_eat(dlist_node_t* phead)
{
	char *pdata = NULL;
	dlist_node_t *pnode = phead->pprev;
	if(pnode->pprev == phead)
		return pdata;
	pnode->pprev->pnext = pnode->pnext;
	pnode->pnext->pprev = pnode->pprev;
	pdata = pnode->pdata;
	free(pnode);
	
	return pdata;
}

void circle_dlist_end(dlist_node_t* phead)
{
    dlist_node_t *pnode = phead->pnext;
    dlist_node_t *temp_node = NULL;
    while(pnode != phead)
	{
        temp_node = pnode;
        pnode = pnode->pnext;
		if(temp_node->pdata!=NULL)
			free(temp_node->pdata);
        free(temp_node);
	}
    free(phead);
}

void circle_dlist_print(dlist_node_t* phead)
{
    dlist_node_t *pnode = phead;
    while(pnode->pnext != phead)
    {
        pnode = pnode->pnext;
        printf("%s",pnode->pdata);
    }
}
