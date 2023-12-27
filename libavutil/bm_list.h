#ifndef BM_LIST_H
#define BM_LIST_H


struct list_head {
    struct list_head *prev, *next;
};

#define LIST_HEAD_INIT(name) {&(name), &(name)}
#define LIST_HEAD(name) struct list_head name= LIST_HEAD_INIT(name)

/* Get an entry from the list
 *  ptr - the address of this list_head element in "type"
 *  type - the data type that contains "member"
 *  member - the list_head element in "type"
 */
#define list_entry(ptr, type, member) \
    ((type *)((uint64_t)(ptr) - (uint64_t)offsetof(type, member)))

/* Get each entry from a list
 *  pos - A structure pointer has a "member" element
 *  head - list head
 *  member - the list_head element in "pos"
 *  type - the type of the first parameter
 */
#define list_for_each_entry(pos, head, member, type)            \
    for (pos = list_entry((head)->next, type, member);          \
         &pos->member != (head);                                \
         pos = list_entry(pos->member.next, type, member))

#define list_for_each_entry_safe(pos, n, head, member, type)    \
    for (pos = list_entry((head)->next, type, member),            \
         n = list_entry(pos->member.next, type, member);        \
         &pos->member != (head);                                \
         pos = n, n = list_entry(n->member.next, type, member))

#define list_empty(entry) ((entry)->next == (entry))
#define list_for_each(pos, head) for(pos=(head)->next; pos!=(head); pos=pos->next)

static inline void list_init(struct list_head *entry) {
    entry->prev = entry->next = entry;
}

static inline void list_add(struct list_head *entry, struct list_head *head) {
    entry->next = head->next;
    entry->prev = head;

    head->next->prev = entry;
    head->next = entry;
}

static inline void list_add_tail(struct list_head *entry,
                                 struct list_head *head) {
    entry->next = head;
    entry->prev = head->prev;

    head->prev->next = entry;
    head->prev = entry;
}

static inline void list_del(struct list_head *entry) {
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
    entry->next = entry->prev = NULL;
}

static inline int list_length(struct list_head *entry) {
    int n = 0;
    struct list_head *pos;
    list_for_each(pos, entry) {
        n++;
    }
    return n;
}

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *mptr = (ptr);    \
        (type *)( (char *)mptr - offsetof(type,member) );})


#endif
