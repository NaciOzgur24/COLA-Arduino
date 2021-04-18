int incomingByte=0;
int fileNumber=1;
int noOfChars;
long int valToWrite;
char activityToLog;
long int x;
long int startLogTime = 0;

void setup() {
	Serial.begin(9600);	              // opens serial port, sets data rate to 9600 bps
        Serial.print("IPA");                  // sets the vdip to use ascii numbers 
                                              //(so I can read them in the code easily!)
        Serial.print(13, BYTE);               // return character to tell vdip its end of message
}

void loop() {              
	if (Serial.available() > 0) {        // read the incoming byte
		incomingByte = Serial.read();

if (incomingByte=='1'){                      // if it receives a 1
  Serial.print("OPW LOG%");                  // open to write creates a file - named
  Serial.print(fileNumber);                  // LOG%1.TXT first time round - .TXT is for the computer
  Serial.print(".TXT");                      // I have used the % sign in the name so I can 
                                             //search the disk for log files that it has created 
                                             //so as not to overwrite any that may be on already
  Serial.print(13, BYTE);                    // return character

}

if (incomingByte=='2'){                      // if it receives a 2
               Serial.print("CLF LOG%");     // it closes the file
               Serial.print(fileNumber);     // LOG%1.TXT
               Serial.print(".TXT");
               Serial.print(13, BYTE);       // return character

	}

if (incomingByte=='3'){                      // if it receives 3
               Serial.print("DIR");          // lists the directory on the memory stick
               Serial.print(13, BYTE);       // which will now include the file LOG%1.TXT
	}                                    


if (incomingByte=='5'){
               activityToLog='P';                  // I will be using different letters to label my time stamps
               valToWrite=millis() - startLogTime; // time since we started logging
               noOfChars=1;
               x=valToWrite;                       // need to copy valToWrite as getting no of characters will consume it
               while (x>= 10){                     // counts the characters in the number
                 noOfChars++;                      // thanks to D Mellis for this bit
                 x/=10;     
               }
               noOfChars +=2;                      //add 2 to the num as will also write the letter P and a return character

               Serial.print("WRF ");               //write to file (file needs to have been opened to write first)
               Serial.print(noOfChars);            //needs to then be told how many characters will be written
               Serial.print(13, BYTE);             //return to say command is finished
               Serial.print(activityToLog);        //followed by the info to write
               Serial.print(valToWrite);
               Serial.print(13, BYTE);             //write a return to the contents of the file (so each entry appears on a new line)
	}

if (incomingByte=='6'){                          
  fileNumber++;                                    //so we can create other files
}

if (incomingByte=='7'){ 
  if (fileNumber>0){                               //and look at previous ones
    fileNumber--;
  }
}

if (incomingByte=='8'){                        
               Serial.print("RD LOG%");            //reads the contents of named file
               Serial.print(fileNumber);           
               Serial.print(".TXT");
               Serial.print(13, BYTE);
	}}


if (incomingByte=='9'){   
  startLogTime=millis();                          // reset timing
}

}