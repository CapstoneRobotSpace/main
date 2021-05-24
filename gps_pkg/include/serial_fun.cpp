#include "serial.cpp"
#include <vector>
#include <iomanip>
using namespace std;

string get_data(Serial &gps_serial){
	string result = "";
	char read_buf [MAX_BUF];
    char *read_index = read_buf;
    int buf_stack = 0;
    bool flag = true;
    bool colon = false;
    bool data_start = true;
    memset(read_buf, '\0', MAX_BUF*sizeof(char));
    while(data_start){
		int read_byte = gps_serial.sread(read_index, sizeof(char));
		if ( read_byte< 0) {
			printf("Error reading: %s", strerror(errno));
			data_start = false;
		}
		else if(read_byte == 1 && (*read_index) == '$'){
			memset(read_buf, '\0', MAX_BUF*sizeof(char));
			read_buf[0] = '$';
			read_index = read_buf;
			read_index++;
			buf_stack = 1;
			flag = true;
		}
		else if(flag){
			if(read_byte == 1){
				if(buf_stack<MAX_BUF){
					if((*read_index) == '\n'){
						*read_index = '\0';
						result = read_buf;
						memset(read_buf, '\0', MAX_BUF*sizeof(char));
						data_start = false;
					}
					read_index++;
					buf_stack++;
				}
				else{
					cout<<"Data Over Flow!!"<<endl;
					memset(read_buf, '\0', sizeof(MAX_BUF));
					read_index = read_buf;
					buf_stack = 0;
					flag = false;
				}	
			}
		}
	}
	return result.substr(1);
}

vector<int> get_comma_position(string &data){
	vector<int> result;
	for(int i =0;i<data.size();i++){
		if(data[i] == ',')
			result.push_back(i);
	}
	return result;
}

vector<float> get_gps(Serial &serial){
	string gnrmc = "";
	string gngga = "";
	string temp;
	vector<float> result;
	vector<int> comma,gngga_c,gnrmc_c;
	bool is_get = false;
	while(!is_get){
		temp = get_data(serial);
		comma = get_comma_position(temp);
		if(temp.substr(0,comma[0]) == "GNRMC"){
			gnrmc = temp;
			gnrmc_c = comma;
		}
		else if(temp.substr(0,comma[0]) == "GNGGA"){
			gngga = temp;
			gngga_c = comma;
		}
		if(gnrmc.size() && gngga.size())
			is_get = true;
	}
	cout<<gnrmc<<endl<<gngga<<endl;
	if(gnrmc.size()){
		if(gnrmc[gnrmc_c[1] + 1] == 'A'){
			cout<<"valid data"<<endl;
			if(gngga.size()>18){
				float time = stof(gngga.substr(gngga_c[0]+1,gngga_c[1] - gngga_c[0]).c_str());
				float latitude = stof(gngga.substr(gngga_c[1]+1,2)) + stof(gngga.substr(gngga_c[1]+3,gngga_c[2] - gngga_c[1] -2).c_str())/60;
				float logtitude = stof(gngga.substr(gngga_c[3]+1,3)) + stof(gngga.substr(gngga_c[3]+4,gngga_c[4] - gngga_c[3] -3).c_str())/60;
				float altitude = stof(gngga.substr(gngga_c[8]+1,gngga_c[9] - gngga_c[8]).c_str());
				cout<<setprecision(10);
				cout<<"time : "<<time<<endl;
				cout<<"latitude : "<<latitude<<endl;
				cout<<"logtitude : "<<logtitude<<endl;
				cout<<"altitude : "<<altitude<<endl;
				result.push_back(time);
				result.push_back(latitude);
				result.push_back(logtitude);
				result.push_back(altitude);
			}
		}
		else{
			cout<<"invalid data"<<endl;
		}
	}
	return result;
}
