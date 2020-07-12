
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef RING_BUFFER_H
#define	RING_BUFFER_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    typedef struct {
        uint8_t size;
        uint8_t head;
        uint16_t * buff;
    } uint_16_t_ring_buffer_t;
#define RING_BUFFER_INIT(name,size) uint_16_t_ring_buffer_t name = { size, 0, uint16_t[size] }
//    {b.buff = malloc(sizeof(uint16_t) * size); b.size = size; b.head = 0;}
#define RING_BUFFER_PUT(b,x)        {b.buff[b.head] = x; b.head++; if (b.head == b.size) b.head = 0;}
#define RING_BUFFER_LAST(b)         (b.buff[b.head - 1])
#define RING_BUFFER_GET(b,i)        (b.buff[i%b.size])
#define RING_BUFFER_END(b)          (b.head == b.size)
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

