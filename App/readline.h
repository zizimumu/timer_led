#ifndef READ_LINE_H
#define READ_LINE_H



struct cmd_tbl_s {
	char		*name;		/* Command Name			*/
	int		maxargs;	/* maximum number of arguments	*/
	int		repeatable;	/* autorepeat allowed?		*/
					/* Implementation function	*/
	int		(*cmd)(struct cmd_tbl_s *, int, int, char *[]);
	char		*usage;		/* Usage message	(short)	*/
	char		*help;		/* Help  message	(long)	*/

};

typedef struct cmd_tbl_s	cmd_tbl_t;

#ifndef NULL
#define NULL ((void *)0)
#endif

extern void usartCharSend(unsigned char charData);
extern  unsigned char usartCharGet(void);
extern void usartStrSend(char * str);


#define s_getchar usartCharGet
#define s_putstring  usartStrSend
#define s_putchar  usartCharSend

#endif
