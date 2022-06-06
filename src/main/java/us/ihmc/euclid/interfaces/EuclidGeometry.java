package us.ihmc.euclid.interfaces;

public interface EuclidGeometry
{
   boolean epsilonEquals(Object object, double epsilon);
   
   boolean geometricallyEquals(Object object, double epsilon);
   
   String toString(String format);
}