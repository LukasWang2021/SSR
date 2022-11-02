#include "buffer_manager.h"
#include <string.h>

/*push one item per operation*/
int32_t push_circle_buff_item(buffer_info_t *circle_buff_info, char *circle_buff, char *push_ptr)
{
    if(is_buff_full(circle_buff_info))
    {
        return -1;
    }

    memcpy(circle_buff+circle_buff_info->tail_index_offset,push_ptr,circle_buff_info->element_length);
    circle_buff_info->tail_index_offset = (circle_buff_info->tail_index_offset + circle_buff_info->element_length)%circle_buff_info->buff_length;
    return circle_buff_info->element_length;
}

/*push all items per operation*/
int32_t push_circle_buff_bundle(buffer_info_t *circle_buff_info, char *circle_buff,
                                buffer_info_t *local_buff_info,  char *local_buff)
{
    int iir;
    int iiw;
    int oor;
    int oow;
/*
    iir = circle_buff_info->head_index_offset;
    iiw = circle_buff_info->tail_index_offset;
    oor = local_buff_info->head_index_offset;
    oow = local_buff_info->tail_index_offset;
    */
    //printf("iir,iiw,oor,oow:%d %d %d %d\r\n",iir,iiw,oor,oow);
    iir = local_buff_info->head_index_offset;
    iiw = local_buff_info->tail_index_offset;
    oor = circle_buff_info->head_index_offset;
    oow = circle_buff_info->tail_index_offset;
    if(iir < iiw)
    {
        if((circle_buff_info->buff_length-oow)>(iiw-iir))
        {
            /*
             * in      |    R------W |temp_var_info
             * local   |   W         |
             * res     |   -------W  |
             */
            //printf("buff_len:%d,data_len:%d\r\n",1024*1024,iiw-iir);
            memcpy(&(circle_buff[oow]), &local_buff[iir], (iiw-iir));

        }else{
            /*
             * in      |    R------W |
             * local   |        W    |
             * res     |-W      -----|
             */
            memcpy(&circle_buff[oow], &local_buff[iir], (circle_buff_info->buff_length-oow));
            memcpy(&circle_buff[0], &local_buff[iir+(circle_buff_info->buff_length-oow)], iiw-iir-(circle_buff_info->buff_length-oow));

        }
        local_buff_info->head_index_offset = local_buff_info->tail_index_offset;
        circle_buff_info->tail_index_offset = (circle_buff_info->tail_index_offset+(iiw-iir))%circle_buff_info->buff_length;
        return (iiw-iir);

    }else{

        if((circle_buff_info->buff_length-oow)>=(local_buff_info->buff_length-iir+iiw))
        {
            /*
             * in      |+++W     R---|
             * local   |      W      |
             * res     |      ---+++W|
             */
            memcpy(&circle_buff[oow], &local_buff[iir], (local_buff_info->buff_length-iir));
            memcpy(&circle_buff[oow+(local_buff_info->buff_length-iir)], &local_buff[0], iiw);

        }else if((circle_buff_info->buff_length-oow)>=(local_buff_info->buff_length-iir)){
            /*
             * in      |+++W     R---|
             * local   |        W    |
             * res     |+W      ---++|
             */
            memcpy(&circle_buff[oow], &local_buff[iir], (local_buff_info->buff_length-iir));
            memcpy(&circle_buff[oow+(local_buff_info->buff_length-iir)], &local_buff[0], (circle_buff_info->buff_length-oow)-(local_buff_info->buff_length-iir));
            memcpy(&circle_buff[0], &local_buff[(circle_buff_info->buff_length-oow)-(local_buff_info->buff_length-iir)], (local_buff_info->buff_length-iir+iiw)-(circle_buff_info->buff_length-oow));

        }else if((circle_buff_info->buff_length-oow)<(local_buff_info->buff_length-iir)){
            /*
             * in      |+++W     R---|
             * local   |           W |
             * res     |-+++W      --|
             */
            memcpy(&circle_buff[oow], &local_buff[iir], (circle_buff_info->buff_length-oow));
            memcpy(&circle_buff[0], &local_buff[iir+(circle_buff_info->buff_length-oow)], (local_buff_info->buff_length-iir)-(circle_buff_info->buff_length-oow));
            memcpy(&circle_buff[(local_buff_info->buff_length-iir)-(circle_buff_info->buff_length-oow)], &local_buff[0], iiw);

        }
        local_buff_info->head_index_offset = local_buff_info->tail_index_offset;
        circle_buff_info->tail_index_offset = (circle_buff_info->tail_index_offset+(local_buff_info->buff_length-iir+iiw))%circle_buff_info->buff_length;
        return (local_buff_info->buff_length-iir+iiw);
    }
    return -1;
}


int32_t pull_circle_buff_item(buffer_info_t *circle_buff_info, char *circle_buff, char *pull_ptr)
{
    //printf("head:%d tail:%d\n",circle_buff_info->head_index_offset, circle_buff_info->tail_index_offset);
    
    if(circle_buff_info->head_index_offset != circle_buff_info->tail_index_offset)
    {
        //printf("local_buff_info.r:%d local_buff_info.w:%d\r\n",local_buff_info.r,local_buff_info.w);
        memcpy(pull_ptr,&circle_buff[circle_buff_info->head_index_offset],circle_buff_info->element_length);
        circle_buff_info->head_index_offset = (circle_buff_info->head_index_offset+circle_buff_info->element_length)%circle_buff_info->buff_length;
        //printf("head:%d tail:%d\n",circle_buff_info->head_index_offset, circle_buff_info->tail_index_offset);
        return 0;
    }
    return -1;//buffer empty
}

int32_t get_circle_buff_occupied(buffer_info_t *circle_buff_info)
{
	int32_t len = (circle_buff_info->tail_index_offset - circle_buff_info->head_index_offset + circle_buff_info->buff_length)%circle_buff_info->buff_length;
	return len/circle_buff_info->element_length;
}

int32_t pull_circle_buff_all(buffer_info_t *circle_buff_info, char *circle_buff, char *pull_ptr, int32_t req_nums)
{
	int32_t nums = 0;
	int32_t res = 0;
	int32_t cnt = 0;
    nums = get_circle_buff_occupied(circle_buff_info);
	if(nums < req_nums)
	{
		while(cnt < nums)
	    {
	        
	        memcpy(pull_ptr+cnt*circle_buff_info->element_length,&circle_buff[circle_buff_info->head_index_offset],circle_buff_info->element_length);
	        circle_buff_info->head_index_offset = (circle_buff_info->head_index_offset+circle_buff_info->element_length)%circle_buff_info->buff_length;
			cnt++;
	    }
		res = nums;
	}else{
		while(cnt < req_nums)
	    {
	        memcpy(pull_ptr+cnt*circle_buff_info->element_length,&circle_buff[circle_buff_info->head_index_offset],circle_buff_info->element_length);
	        circle_buff_info->head_index_offset = (circle_buff_info->head_index_offset+circle_buff_info->element_length)%circle_buff_info->buff_length;
			cnt++;
	    }
		res = req_nums;
	}
	return res;
}


/*pull all data per operation*/
int32_t pull_circle_buff_bundle(buffer_info_t *circle_buff_info, char *circle_buff,
                                buffer_info_t *local_buff_info, char *local_buff)
{
    int iir;
    int iiw;
    int oor;
    int oow;

    iir = circle_buff_info->head_index_offset;
    iiw = circle_buff_info->tail_index_offset;
    oor = local_buff_info->head_index_offset;
    oow = local_buff_info->tail_index_offset;
    //printf("iir,iiw,oor,oow:%d %d %d %d\r\n",iir,iiw,oor,oow);

    if(iir<iiw)
    {
        if((local_buff_info->buff_length-oow)>(iiw-iir))
        {
            /*
             * in      |    R------W |temp_var_info
             * local   |   W         |
             * res     |   -------W  |
             */
            //printf("buff_len:%d,data_len:%d\r\n",1024*1024,iiw-iir);
            memcpy(&(local_buff[oow]), &circle_buff[iir], (iiw-iir));

        }else{
            /*
             * in      |    R------W |
             * local   |        W    |
             * res     |-W      -----|
             */
            memcpy(&local_buff[oow], &circle_buff[iir], (local_buff_info->buff_length-oow));
            memcpy(&local_buff[0], &circle_buff[iir+(local_buff_info->buff_length-oow)], iiw-iir-(local_buff_info->buff_length-oow));

        }
        circle_buff_info->head_index_offset = circle_buff_info->tail_index_offset;
        local_buff_info->tail_index_offset = (local_buff_info->tail_index_offset+(iiw-iir))%local_buff_info->buff_length;
        return (iiw-iir);

    }else{

        if((local_buff_info->buff_length-oow)>=(circle_buff_info->buff_length-iir+iiw))
        {
            /*
             * in      |+++W     R---|
             * local   |      W      |
             * res     |      ---+++W|
             */
            memcpy(&local_buff[oow], &circle_buff[iir], (circle_buff_info->buff_length-iir));
            memcpy(&local_buff[oow+(circle_buff_info->buff_length-iir)], &circle_buff[0], iiw);

        }else if((local_buff_info->buff_length-oow)>=(circle_buff_info->buff_length-iir)){
            /*
             * in      |+++W     R---|
             * local   |        W    |
             * res     |+W      ---++|
             */
            memcpy(&local_buff[oow], &circle_buff[iir], (circle_buff_info->buff_length-iir));
            memcpy(&local_buff[oow+(circle_buff_info->buff_length-iir)], &circle_buff[0], (local_buff_info->buff_length-oow)-(circle_buff_info->buff_length-iir));
            memcpy(&local_buff[0], &circle_buff[(local_buff_info->buff_length-oow)-(circle_buff_info->buff_length-iir)], (circle_buff_info->buff_length-iir+iiw)-(local_buff_info->buff_length-oow));

        }else if((local_buff_info->buff_length-oow)<(circle_buff_info->buff_length-iir)){
            /*
             * in      |+++W     R---|
             * local   |           W |
             * res     |-+++W      --|
             */
            memcpy(&local_buff[oow], &circle_buff[iir], (local_buff_info->buff_length-oow));
            memcpy(&local_buff[0], &circle_buff[iir+(local_buff_info->buff_length-oow)], (circle_buff_info->buff_length-iir)-(local_buff_info->buff_length-oow));
            memcpy(&local_buff[(circle_buff_info->buff_length-iir)-(local_buff_info->buff_length-oow)], &circle_buff[0], iiw);

        }
        circle_buff_info->head_index_offset = circle_buff_info->tail_index_offset;
        local_buff_info->tail_index_offset = (local_buff_info->tail_index_offset+(circle_buff_info->buff_length-iir+iiw))%local_buff_info->buff_length;
        return (circle_buff_info->buff_length-iir+iiw);
    }
    return -1;
}

int32_t is_buff_full(buffer_info_t *circle_buff_info)
{
    if(((circle_buff_info->tail_index_offset+circle_buff_info->element_length)%circle_buff_info->buff_length) == circle_buff_info->head_index_offset)
    {
        return 1;
    }
    return 0;
}

int32_t is_buff_empty(buffer_info_t *circle_buff_info)
{
    if(circle_buff_info->tail_index_offset
             == circle_buff_info->head_index_offset)
    {
        return 1;
    }
    return 0;
}

int buffer_clear(buffer_info_t *circle_buff_info)
{
	circle_buff_info->head_index_offset = 0;
	circle_buff_info->tail_index_offset = 0;
    return 0;
}

int buffer_lock(buffer_info_t *circle_buff_info)
{
	int res = 0;
    circle_buff_info->mtx.lock();
	//res = pthread_mutex_lock(&circle_buff_info->mutex);  // 为保证条件变量不会因为多线程混乱，所以先加锁
	return res;
}

void buffer_unlock(buffer_info_t *circle_buff_info)
{
	//pthread_mutex_unlock(&circle_buff_info->mutex);
    circle_buff_info->mtx.unlock();
}

