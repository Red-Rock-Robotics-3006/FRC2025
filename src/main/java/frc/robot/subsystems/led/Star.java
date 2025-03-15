package frc.robot.subsystems.led;

import java.util.Random;

import edu.wpi.first.wpilibj.util.Color;

import javax.lang.model.util.ElementScanner14;

public class Star 
    {
        private boolean increasing;
        private boolean lit;
        private float frequency;
        private Random randomizer;
        private float temperature;
        private float red, green, blue;
        private float temperatureSpeed;
        private float lowTemp, highTemp;
        private float cycleTime, cycleVariation;



        public Star(float lowTemp, float highTemp, float cycleTime, float cycleVariation, float frequency, int seed)
        {
            this.frequency = frequency;
            this.lit = false;
            this.increasing = false;
            this.randomizer = new Random(seed);
            this.temperatureSpeed = cycleTime;
            this.temperature = lowTemp;
            this.lowTemp = lowTemp;
            this.highTemp = highTemp;
            this.cycleTime = cycleTime;
            this.cycleVariation = cycleVariation;
        }

        public Color getTemperatureColor()
        {
            float temptemp = this.temperature/100;
            if(this.temperature == 0) temptemp = 1;
            Color outColor;
            if (lit)
            {
                if(temptemp <= 66)
                {
                    red = 255f;
                } 
                else
                {
                    red = 329.698727446f * (float)Math.pow(temptemp - 60f, -0.1332047592);
                    red = (float)Math.min(red, 255);
                    red = (float)Math.max(red, 0);
                }

                if( temptemp <= 66)
                {
                    green = 99.4708025861f * (float)Math.log(temptemp) - 161.1195681661f;
                    green = (float)Math.min(green, 255);
                    green = (float)Math.max(green, 0);
                }
                else
                {
                    green = 288.1221695283f * (float)Math.pow(temptemp-60, -0.0755148492);
                    green = (float)Math.min(green, 255);
                    green = (float)Math.max(green, 0);
                }
                if(temptemp >= 66)
                {
                    blue = 255f;
                }
                else
                {
                    blue = 138.5177312231f * (float)Math.log(temptemp-10) - 305.0447927307f;
                    blue = (float)Math.min(blue, 255);
                    blue = (float)Math.max(blue, 0);
                }
                //outColor = new Color(red/255.0, green/255.0*4.0, blue/255.0/4);
                float[] hsvvalues = new float[3];
                java.awt.Color.RGBtoHSB((int)red, (int)green, (int)blue, hsvvalues);
                hsvvalues[2]*=Math.pow(((temperature-lowTemp)/(highTemp-lowTemp)),1);
                hsvvalues[2] = Math.max(hsvvalues[2], 0.01f);
                outColor = Color.fromHSV((int)(hsvvalues[0]*180), (int)(hsvvalues[1]*255), (int)(hsvvalues[2]*255));
                if(temperature>=highTemp) increasing = false;
                if(increasing) temperature +=temperatureSpeed; else temperature -= temperatureSpeed;
                if(temperature<=lowTemp)  lit = false;
            } 
            else if(randomizer.nextFloat()<frequency)
            {
                lit = true;
                increasing = true;
                outColor = new Color(0, 0, 1);
                temperatureSpeed = randomizer.nextFloat(cycleTime-cycleVariation/2f, cycleTime+cycleVariation/2f);
            } else outColor = new Color(0, 0, 1);
            return outColor;

        }
        
    }