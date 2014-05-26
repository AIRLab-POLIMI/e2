#include "ReadSonarBase.h"

const char ReadSonarBase::response_ok_header[]="+OK ";
const char ReadSonarBase::response_err_header[]="-ERR";
const char ReadSonarBase::response_dbg_header[]="%DBG";
const char ReadSonarBase::response_meas_header[]="+MEA";


const char* ReadSonarDeviceException::what() const throw(){
  return "Sonar port open error!";
}

ReadSonarBase::ReadSonarBase(float to_meter)
		throw (ReadSonarDeviceException){
	
	pack_n=0;	
	measure=NULL;
	buffer=NULL;
	tmp_buf=NULL;
	this->to_meter=to_meter;
	
	measure=new float[ReadSonarBase::n_sonar];
	tmp_buf=new char[ReadSonarBase::max_buf_tmp];
	for(unsigned int i=0;i<ReadSonarBase::n_sonar;i++){
		measure[i]=-1;
	}
	
}

ReadSonarBase::~ReadSonarBase(){
	if(measure)delete [] measure;
	if(tmp_buf)delete [] tmp_buf;
}

unsigned int ReadSonarBase::getLastPackNum(){
	return pack_n;
}

float ReadSonarBase::getMeasure(unsigned int index){
	if(!measure)return -1;
	if(index<0)return -1;
	if(index>=n_sonar)return -1;
	//printf("Indice:%d - %d",index,measure[index]);
	return measure[index];
}

int ReadSonarBase::parseLine(){
	int parsed_type = ReadSonarBase::parse_err;
	if(buffer->getLineCount()<=0)return parsed_type;
	
	int len=buffer->removeLine(tmp_buf,max_buf_tmp);
	if(len<=0)return parsed_type;
	if(tmp_buf[len-1]=='\n')tmp_buf[len-1]='\0';
	
	if(strncmp(tmp_buf,ReadSonarBase::response_ok_header,ReadSonarBase::response_header_len)==0){
		parsed_type = ReadSonarBase::parse_ok;
	}
	else if(strncmp(tmp_buf,ReadSonarBase::response_err_header,ReadSonarBase::response_header_len)==0){
		parsed_type = ReadSonarBase::parse_response_err;
	}
	else if(strncmp(tmp_buf,ReadSonarBase::response_dbg_header,ReadSonarBase::response_header_len)==0){
		parsed_type = ReadSonarBase::parse_dbg;
	}
	else if(strncmp(tmp_buf,ReadSonarBase::response_meas_header,ReadSonarBase::response_header_len)==0){
		std::vector<std::string> tokens;
		tokenize(std::string(tmp_buf),tokens,",");
		if(tokens.size()!=7){
			parsed_type = ReadSonarBase::parse_err;
			return parsed_type;
		}
		int bank,offset;
		bank = atoi(tokens[2].c_str());
		pack_n = atoi(tokens[1].c_str());
		
		
		//printf("\nSonarExpert:: Buf=%s\n" ,std::string(tmp_buf).c_str());
		
		switch(bank){
			case 9:
				offset=0;
				parsed_type=ReadSonarBase::parse_meas_b1;
				break;
			case 12:
				offset=1;
				parsed_type=ReadSonarBase::parse_meas_b2;
				break;
			default:
				parsed_type=ReadSonarBase::parse_err;
				return parsed_type;
				break;
		}
		
		for (int i=0;i<4;i++){
			measure[i*2+offset]=to_meter*(float)atoi(tokens[i+3].c_str());
			//printf("\nSonarExpert:: Token %d =%s\n",i ,tokens[i+3].c_str());
			
			
		}
	}
	return parsed_type;
}

char * ReadSonarBase::getParsedLine(){
	//call it only after parse line!!!
	return tmp_buf;
}


unsigned int ReadSonarBase::getLineToParseNum(){
	return buffer->getLineCount();
}
