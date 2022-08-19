#ifndef CIRCLE_DUL_LIST_H_
#define CIRCLE_DUL_LIST_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct list_node{
	char *pdata;
	struct list_node *pprev;
	struct list_node *pnext;
}dlist_node_t;

/**
 * @brief Initialize a doubly linked list.
 * @retval Head of linked list.
 */
extern dlist_node_t *circle_dlist_init(void);

/**
 * @brief Insert one node to the tail of the doubly linked list.
 * @param [in] Head of linked list.
 * @param [in] data of the new node
 * @retval Head of linked list.
 */
extern dlist_node_t *circle_dlist_tail_insert(dlist_node_t *phead, char *pdata);

/**
 * @brief Get the length of the doubly linked list.
 * @param [in] Head of linked list.
 * @retval Length of the doubly linked list.
 */
extern int circle_dlist_get_nums(dlist_node_t *phead);

/**
 * @brief Get the first node of the doubly linked list.
 * @param [in] Head of linked list.
 * @retval Pointer of the first node data.
 */
extern char *circle_dlist_head_eat(dlist_node_t* phead);

/**
 * @brief Get the last node of the doubly linked list.
 * @param [in] Head of linked list.
 * @retval Pointer of the last node data.
 */
extern char *circle_dlist_tail_eat(dlist_node_t* phead);

/**
 * @brief Free the doubly linked list.
 * @param [in] Head of linked list.
 * @retval void.
 */
extern void circle_dlist_end(dlist_node_t* phead);

/**
 * @brief Print every node of the doubly linked list.
 * @param [in] Head of linked list.
 * @retval void.
 */
extern void circle_dlist_print(dlist_node_t* phead);

#ifdef __cplusplus
}
#endif

#endif