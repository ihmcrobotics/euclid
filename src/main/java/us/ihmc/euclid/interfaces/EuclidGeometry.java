package us.ihmc.euclid.interfaces;

public interface EuclidGeometry
{
   boolean epsilonEquals(Object obj, double epsilon);
   
   boolean geometricallyEquals(Object obj, double epsilon);
   
   // rename.
   String makeString();
   
   String makeString(String format);
}
