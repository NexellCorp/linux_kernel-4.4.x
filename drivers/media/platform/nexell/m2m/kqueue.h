/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __KQUEUE_H__
#define __KQUEUE_H__

struct queue_entry {
	struct list_head head;
	void *data;
};

struct common_queue {
	struct list_head head;
	spinlock_t slock;
	unsigned long flags;

	int buf_count;

	void (*init)(struct common_queue *);
	void (*enqueue)(struct common_queue *, struct queue_entry *);
	struct queue_entry *(*dequeue)(struct common_queue *);
	struct queue_entry *(*peek)(struct common_queue *, int pos);
	int (*clear)(struct common_queue *);
	void (*lock)(struct common_queue *);
	void (*unlock)(struct common_queue *);
	int (*size)(struct common_queue *);
};

static void _init_queue(struct common_queue *q)
{
	INIT_LIST_HEAD(&q->head);
	spin_lock_init(&q->slock);
	q->buf_count = 0;
}

static void _enqueue(struct common_queue *q, struct queue_entry *buf)
{
	q->lock(q);
	list_add_tail(&buf->head, &q->head);
	q->buf_count++;
	q->unlock(q);
}

static struct queue_entry *_dequeue(struct common_queue *q)
{
	struct queue_entry *ent = NULL;
	struct nx_video_buf *buf = NULL;

	q->lock(q);
	if (!list_empty(&q->head)) {
		ent = list_first_entry(&q->head, struct queue_entry, head);
		buf = (struct nx_video_buf *)(ent->data);
		list_del_init(&ent->head);
		q->buf_count--;

		if (q->buf_count < 0)
			q->buf_count = 0;
	}
	q->unlock(q);

	return ent;
}

static struct queue_entry *_peekqueue(struct common_queue *q, int pos)
{
	struct queue_entry *ent = NULL;
	int idx = 0;

	q->lock(q);
	if ((q->buf_count > 0) && (q->buf_count >= (pos+1))) {
		list_for_each_entry(ent, &q->head, head) {
			if (idx == pos)
				break;

			idx++;
		}
	}
	q->unlock(q);

	if (idx > pos)
		ent = NULL;

	return ent;
}

static int _clearqueue(struct common_queue *q)
{
	while (q->dequeue(q))
		;
	return 0;
}

static int _sizequeue(struct common_queue *q)
{
	int count = 0;

	q->lock(q);
	count = q->buf_count;
	q->unlock(q);

	return count;
}

static void _lockqueue(struct common_queue *q)
{
	spin_lock_irqsave(&q->slock, q->flags);
}

static void _unlockqueue(struct common_queue *q)
{
	spin_unlock_irqrestore(&q->slock, q->flags);
}

static void register_queue_func(
	struct common_queue *q,
	void (*init)(struct common_queue *),
	void (*enqueue)(struct common_queue *, struct queue_entry *),
	struct queue_entry *(*dequeue)(struct common_queue *),
	struct queue_entry *(*peek)(struct common_queue *, int pos),
	int (*clear)(struct common_queue *),
	void (*lock)(struct common_queue *),
	void (*unlock)(struct common_queue *),
	int (*size)(struct common_queue *)

	)
{
	q->init = init;
	q->enqueue = enqueue;
	q->dequeue = dequeue;
	q->peek	= peek;
	q->clear = clear;
	q->lock = lock;
	q->unlock = unlock;
	q->size = size;
}

#endif
