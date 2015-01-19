void on_ConfigReg_clicked()
{
    Funkcije* funcc = new Funkcije(this->genio); 

    funcc->SetPattern(0x00);

    //Retrieve information about pixel setup from ui.

    //16 global control bits (GCBits)
    bool GCBits[16]; //Global Control Bits
    int TDAC[3];     // Use this to read in from UI. Convert to bool later.

    TDAC[0] = ui->gtdac1->text().toInt();// TDAC 1
    TDAC[1] = ui->gtdac2->text().toInt(); // TDAC 2
    TDAC[2] = ui->gtdac3->text().toInt(); // TDAC 3
    for(int i=0; i<3; i++)
    {
        if(TDAC[i] > 15 || TDAC[i] < 0) // Check if integer is in 4bit range.
            TDAC[i] = 0;
    }

    // Is this corrrect with MSB and order of bits?
    //TDAC2
    GCBits[0] = 0x08 & TDAC[1];   //TDAC2(0:3) // MSB
    GCBits[1] = 0x04 & TDAC[1];   //TDAC2(0:3)
    GCBits[2] = 0x02 & TDAC[1];   //TDAC2(0:3)
    GCBits[3] = 0x01 & TDAC[1];   //TDAC2(0:3)
    //TDAC 1
    GCBits[4] = 0x08 & TDAC[0];   //TDAC1(0:3) // MSB
    GCBits[5] = 0x04 & TDAC[0];   //TDAC1(0:3)
    GCBits[6] = 0x02 & TDAC[0];   //TDAC1(0:3)
    GCBits[7] = 0x01 & TDAC[0];   //TDAC1(0:3)
    //TDAC 3
    GCBits[12] = 0x08 & TDAC[2];   //TDAC3(0:3) // MSB
    GCBits[13] = 0x04 & TDAC[2];   //TDAC3(0:3)
    GCBits[14] = 0x02 & TDAC[2];   //TDAC3(0:3)
    GCBits[15] = 0x01 & TDAC[2];   //TDAC3(0:3)


    GCBits[8] = gconf.ABEn; // ABEnB

    GCBits[9] = gconf.CompOffNorm; //CompOffBNormal

    GCBits[10] = gconf.CompOffB; //CompOffB

    GCBits[11] = gconf.EnLowPass; //EnLowPass

    //Write configuration into Chip
    // Start from 289 down to 0
    // Send 14 bias DACs (6bit and one spare) with SendDAC in class Funkcije. Bits 289 down to 192
    //
    std::cout << "|289 down to 192>> ";
    for(int i=13; i>=0; i--)
    {
        funcc->SendDAC(gconf.GetbDAC(i), gconf.GetSpare(i)); //SendDac(int, bool Sparebit)
        std::cout << "'";
    }
    std::cout << std::endl << "|191 down to 176>> ";
    // Send 16 control bits. 191 down to 176.
    for(int i=15;i>=0; i--)
    {
        funcc->SendBit(GCBits[i]);
    }
    std::cout << std::endl << "|175 down to 0  >> ";
    // Send configuration for 22 column pairs. Bits 175 down to 0
    for(int i=22; i>0; i--)
    {
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).AnaInj); // AnaInj is last 2 bits
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).Ld); // Ld
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).HBEn); // HBEn
        for(int RO=2; RO>0; RO--)
            funcc->SendBit(H35pixel.GetBits(RO,i).DigInjEn); //DigInjEn is first 2 bits
        std::cout << "'";
    }
    std::cout << std::endl;
    //logit("AnaInjPattern:");
   // logit(H35pixel.GetPatternAnaInj());
    //logit("Ld Pattern:");
    //logit(H35pixel.GetPatternLd());
    //logit("HBEn Pattern:");
    //logit(H35pixel.GetPatternHBEn());
    //logit("DigInj Pattern:");
    //logit(H35pixel.GetPatternDigInj());

    // Set the load bit
    funcc->LoadConf();
    // Send Config
    try
    {
        if(!funcc->SendBuffer())
            throw 3;
        logit("Config written into Chip and loaded.");
    }
    catch(int exception)
    {
        switch(exception)
        {
            case 3: logit("Communication with FPGA failed.");
                break;
            default:
                logit("Unkown Error.");
        }
    }

    delete funcc;

}


void Funkcije::SendBit(bool bit)  // config bit high (true) or low (false)
{
   unsigned char pattern;  //bit patterns for 8 bit registers in xilinx
   pattern = this->Bit;
  // pattern = pattern | PAT_ParEn;
   if(bit)
     {
       pattern = pattern | PAT_SIn;
     }
   std::cout << (int)bit;
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern | PAT_CKRo1; // Clk1 high
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo1; // Clk1 low
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern | PAT_CKRo2; // Clk 2 high
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   pattern = pattern & ~PAT_CKRo2; // Clk2 low
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
}

void  Funkcije::SendLoadConf() // This loads the config bits into the chip
{
   unsigned char pattern;  //bit patterns for 8 bit registers in xilinx

   pattern = this->Bit;
   pattern = pattern | PAT_LdConf; // Send a longer high for load.
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
   //pattern = FormMain -> Bit;
   pattern = this->Bit;             // Ld low
   genio->addpair( 0x41, pattern);
   genio->addpair( 0x41, pattern);
}