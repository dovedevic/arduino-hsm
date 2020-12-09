#include <NoiseSource.h>
#include <SHA256.h>
#include <RNG.h>
#include <Crypto.h>
#include <AES.h>
#include <Hash.h>
#include <P521.h>
#include <Wire.h>
//#include <TransistorNoiseSource.h>

// AES object for symetric crypto
AES256 aes256;


// static key for testing on start up
byte key[32] = {0x00, 0x01, 0x0a, 0x01, 0x00, 0x01, 0x00, 0x01,
                0x00, 0x01, 0x04, 0x01, 0x00, 0x01, 0x00, 0x01};

// SHA256 object for HMAC
SHA256 sha256;
#define HASH_SIZE 32
#define BLOCK_SIZE 64


// Operation enum that is sent between boards telling 
// the crypto board what action to perform
//P521 not implemented at the moment
enum OP {
  _AESENCRYPT,
  _AESDECRYPT,
  _SHA256,
  _RNG,
  _P521PUB,
  _P521PRI
};


void setup() {
  // print for debugging
  Serial.begin(9600);
  
  // Security Module is number 4
  Wire.begin(4);

  //register event
  Wire.onReceive(receiveEvent);

  // set up rng
  //RNG.begin("Crypto Board");

  // add nosie source
  //RNG.addNoiseSource(noise);

  //Debug Message
  Serial.println("Iniializing Crypto Board");
  test();
}

/*
 * Test function o test that the board is running properly on startup
 */
void test(){
  //TODO: write tests for each crypto operation
  
}


void loop() {
  delay(100);
}

void receiveEvent(int x){
  // get the enum from the wire
  OP action = (OP) Wire.read();

  // each dcase calls the relevent function
  switch(action) {
    case _AESENCRYPT:
      //call aesEncrypt
      aesEncrypt();
      break;
    case _AESDECRYPT:
      //call aesDecrypt
      aesDecrypt();
      break;
    case _SHA256:
      //call hash
      hash();
      break;
    case _RNG:
      //call rng
      rng();
      break;
    case _P521PUB:
      // call encryptPub
      encryptPub();
      break;
    case _P521PRI:
      // call encryptPri
      encryptPri();
      break;
    default:
      // error
      Serial.println("Error: Unrecognized OP value, exiting program");
      exit(-1);
      // may need to clear everything depending on implementation
      break;
  }  

}

/*
 * Reads plaintext and uses internal key
 * Writes back the ciphertext
 */
void aesEncrypt(){
  // se up variable to read plaintext
  byte plaintext[32];
  byte ciphertext[32];
  int bytesRead = 0;
  
  // ge plaintext from master
  while(Wire.available()){
    plaintext[bytesRead] =Wire.read();
    bytesRead++;
  }

  //set up encryption
  aes256.setKey(key, aes256.keySize());
  aes256.encryptBlock(ciphertext, plaintext);

   Wire.write(ciphertext, 32);
  
}

/*
 * Reads ciphertext and uses internal key
 * Writes back the plaintext
 */
void aesDecrypt(){
  // se up variable to read plaintext
  byte ciphertext[32];
  byte plaintext[32];
  int bytesRead = 0;
  
  // ge plaintext from master
  while(Wire.available()){
    ciphertext[bytesRead] =Wire.read();
    bytesRead++;
  }

  //set up encryption
  aes256.setKey(key, aes256.keySize());
  aes256.decryptBlock(plaintext, ciphertext);

  // write plaintext back to main board
   Wire.write(plaintext, 32);
  
}

/*
 * Reads data and uses internal key
 * Writes back the resulting HMAC
 */
void hash(){
  
  // initialize variables
  int bytesRead = 0;
  byte data [32];
  byte result[HASH_SIZE];
  
  // reset the HMAC
  sha256.resetHMAC(&key, sizeof(key));

  //get hash data
  while(Wire.available()){
    data[bytesRead] =Wire.read();
    bytesRead++;
  }

  // add
  sha256.update(&data, bytesRead);

  // HMAC data
  sha256.finalizeHMAC(&key, sizeof(key), result, sizeof(result));

  //send back hash value
  Wire.write(result, sizeof(result));
}

/*
 * Currently using Arduinos builtin random
 * Sample the time every time rng is called to generate a psuedo-random seed
 * Reads the number of requested bytes
 * Writes back the number of bytes
 */
void rng(){

  //seed based on time      
  randomSeed(millis());

  // get number of bytes to generate
  int bytesRequested = Wire.read();

  //generate requested number of bytes
  for(int i = 0; i < bytesRequested; i++) {

    // generate random byte
    byte randByte = random(256);

    //debug print random byte
    Serial.println(randByte);

    //send byte back to main board
    Wire.write(randByte);   
  }
  
}

void encryptPub(){
  Serial.println("Public Key algorithm not yet implemented");
  return;
}

void encryptPri(){
  Serial.println("Private Key algorithm not yet implemented");
  return;
}
