#include <er_ti_f28069m_drv8305/er_buffer.h>


ER_Buffer buffer__create(size_t buff_size)
{
    ER_Buffer channel_buff;
    channel_buff.buf = (uint16_t *)malloc(buff_size);
    channel_buff.buff_size = buff_size;
    channel_buff.buff_is_changed= false;
    channel_buff.length = 0;
    return channel_buff;

}

void buffer__add_to_buffer(ER_Buffer *channel_buff, uint16_t value)
{
    // set buffer changed flag
    channel_buff->buff_is_changed = true;

    // Add value to the buffer and move pointer
    if (channel_buff->length == channel_buff->buff_size) {
        buffer__remove_from_buffer(channel_buff, 1);
    }

    channel_buff->buf[(channel_buff->length)++] = value;
}

void buffer__remove_from_buffer (ER_Buffer *channel_buff, uint16_t remove_size){
    if (remove_size > channel_buff->length) {
        remove_size = channel_buff->length;
    }
    channel_buff->length -= remove_size;
    memcpy(channel_buff->buf,
           (channel_buff->buf) + (remove_size),
           channel_buff->length * sizeof(uint16_t));
    channel_buff->buff_is_changed = true;
}

void buffer__destroy(ER_Buffer *channel_buff){
    free(channel_buff->buf);
    free((void *) channel_buff);
}

void buffer__resize(ER_Buffer *channel_buff,size_t size){
    if (size >= channel_buff->buff_size) {
        realloc(channel_buff->buf, size);
        channel_buff->buff_size = size;
    } else {
        buffer__remove_from_buffer(channel_buff, channel_buff->buff_size - size);
        realloc(channel_buff->buf, size);
        channel_buff->buff_size = size;
    }
}

uint16_t buffer__median(ER_Buffer *channel_buff) {
    int n = channel_buff->length;
    uint16_t *array = (uint16_t *)malloc(n);
    memcpy(array, channel_buff->buf, n * sizeof(uint16_t));
    uint16_t temp;
    int i, j;
    // the following two loops sort the array in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(array[j] < array[i]) {
                // swap elements
                temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }
    uint16_t median;
    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        median = ((array[n/2] + array[n/2 - 1]) / 2);
    } else {
        // else return the element in the middle
        median = array[n/2];
    }
    free(array);
    return median;
}
