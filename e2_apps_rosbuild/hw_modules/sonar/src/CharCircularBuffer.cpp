#include "CharCircularBuffer.h"


#if (defined TEST_VERSION) || (defined  DEBUG)
#include <sstream>
#include <iostream>

#endif
CharCircularBuffer::CharCircularBuffer(unsigned int n,char end_line_char){
	buf=new char[n+1];
	this->n=n+1;
	start=0;
	end=0;
	lineCount=0;
	this->end_line_char=end_line_char;
}


CharCircularBuffer::~CharCircularBuffer(){
	delete [] buf;
}

unsigned int CharCircularBuffer::getLineCount(){
	return lineCount;
}
		
int CharCircularBuffer::addChar(char src){
	 //ret n char added (0: error, 1 ok)
	 if(isFull())return 0;
	 buf[end]=src;
	 end=inc(end);
	 if(src==end_line_char)lineCount++;
	 return 1;
 }

int CharCircularBuffer::removeChar(char *dest){//ret n char removed (0: buffer empty)
	if(isEmpty())return 0;
	*dest=buf[start];
	start=inc(start);
	if(*dest==end_line_char)lineCount--;
	return 1;
}

int CharCircularBuffer::addNChar(char *src,unsigned int nx){
//ret n char added (0: error, 1 ok)
	unsigned int disp=n-getCount()-1;//#posti disponibili
	nx=(disp>nx?nx:disp);
	for (int i=0;i<nx;i++){
		if(src[i]==end_line_char)lineCount++;
		buf[end]=src[i];
		end=inc(end);		
	}
	return nx;
}

int CharCircularBuffer::removeNChar(char *dest,unsigned int nx){	
	//ret n char removed (0: buffer empty, 0-n)
	unsigned int disp=getCount();//#posti disponibili
	nx=(disp>nx?nx:disp);
	for (int i=0;i<nx;i++){
		if(buf[start]==end_line_char)lineCount--;
	 	dest[i]=buf[start];
	 	start=inc(start);		
	}
	return nx;
}

int CharCircularBuffer::removeLine(char *dest,unsigned int maxn){
	//ret nchar readed, (max maxn char), terminate string with \0 (instead of \n or in maxn position)
	unsigned int c=0;
	if(lineCount==0)return 0;
	while( (!isEmpty()) && (buf[start]!=end_line_char) && (c<maxn) ){
		dest[c]=buf[start];
		start=inc(start);
		c++;
	}
	dest[c]='\0';
	if(!isEmpty() && buf[start]==end_line_char){
		lineCount--;
		start=inc(start);
	}
	return c;
}

#if (defined TEST_VERSION) || (defined  DEBUG)


std::string CharCircularBuffer::getStringStatus(){
	std::ostringstream ss;
	char str[n];
	unsigned int x;
	unsigned int i=0;
	x=start;
	for (i=0;i<getCount();i++){
		//std::cout<<"I:"<<i<<std::endl;
		str[i]=buf[x];
		x=inc(x);
	}
	str[i]='\0';
	ss<<"#:"<<getCount()<<" s="<<start<<" e="<<end<<" l="<<getLineCount()<<" E?"<<isEmpty()<<" F?"<<isFull()<<" buf=["<<str<<"]\n";
	
	return ss.str();
	
}
#endif

#ifdef TEST_VERSION
int main (){
	CharCircularBuffer *b=new CharCircularBuffer(5);
	unsigned int x;
	char c,s[10];
	
	std::cout<<b->getStringStatus();
	
	std::cout<<b->addChar('a')<<std::endl;
	std::cout<<b->getStringStatus();
	
	std::cout<<b->addChar(end_line_char)<<std::endl;
	std::cout<<b->getStringStatus();
	
	std::cout<<b->removeChar(&c);
	std::cout<<" "<<c<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->removeChar(&c);
	std::cout<<" "<<c<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->addNChar((char *)"1\n2\n3asldal",4 )<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->addChar(end_line_char)<<std::endl;
	std::cout<<b->getStringStatus();

	x=b->removeNChar(s,4);
	s[x]='\0';
	std::cout<<x<<" "<<s<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->addNChar((char *)"123\ndfsadasdasda",10)<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->removeLine(s,5);
	std::cout<<" "<<s<<std::endl;
	std::cout<<b->getStringStatus();

	std::cout<<b->removeLine(s,5);
	std::cout<<" "<<s<<std::endl;
	std::cout<<b->getStringStatus();
	
	delete b;
	return 0;
}

#endif
