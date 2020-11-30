#include <AuthenticatedCipher.h>
#include <BLAKE2b.h>
#include <NoiseSource.h>
#include <ChaCha.h>
#include <XOF.h>
#include <BLAKE2s.h>
#include <Poly1305.h>
#include <SHA3.h>
#include <KeccakCore.h>
#include <ChaChaPoly.h>
#include <SHA256.h>
#include <RNG.h>
#include <EAX.h>
#include <GHASH.h>
#include <Cipher.h>
#include <Crypto.h>
#include <SHA512.h>
#include <BlockCipher.h>
#include <SHAKE.h>
#include <AES.h>
#include <BigNumberUtil.h>
#include <Ed25519.h>
#include <GCM.h>
#include <Curve25519.h>
#include <XTS.h>
#include <GF128.h>
#include <CTR.h>
#include <Hash.h>
#include <P521.h>
#include <OMAC.h>


#include <Wire.h>
//#include <TransistorNoiseSource.h>

// AES object
AES256 aes256;

// statically set aes key for now
byte key[32] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
                0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01};

// SHA256 object
SHA256 sha256;

// Noise source to seed the random number generator.
//TransistorNoiseSource noise(A1);

byte rand_data[32];

// Operation enum that is sent between boards telling 
// the crypto board what action to perform
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
  
  // Security Module is number 1
  Wire.begin(1);

  //register event
  Wire.onReceive(receiveEvent);

  // set up rng
  RNG.begin("Hello");

  // add nosie source
  //RNG.addNoiseSource(noise);
}



void loop() {
  delay(100);
}

void receiveEvent(int x){
  // get the enum from the wire
  OP action = (OP) Wire.read();

  switch(action) {
    case _AESENCRYPT:
      aesEncrypt();
      break;
    case _AESDECRYPT:
      aesDecrypt();
      break;
    case _SHA256:
      //call hash
      break;
    case _RNG:
      //call rng
      rng();
      break;
    case _P521PUB:
      // call encrypt_pub
      break;
    case _P521PRI:
      // call encrypt_pri
      break;
    default:
      // error
      // may need to clear everything depending on implementation
      break;
  }  

}

void rng(){
  // loop needs to be run periodically
  RNG.loop();

  // get rng
  if (RNG.available(sizeof(rand_data))) {
        RNG.rand(rand_data, sizeof(rand_data));
    }
      
  //send rng to master
  Wire.write(rand_data, sizeof(rand_data));
  
}

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

void hash(){
  //get data


  //hash data
  


  //send back hash value
  


}
