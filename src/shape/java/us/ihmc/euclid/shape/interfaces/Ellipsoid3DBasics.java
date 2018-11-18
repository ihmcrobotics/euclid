package us.ihmc.euclid.shape.interfaces;

public interface Ellipsoid3DBasics extends Ellipsoid3DReadOnly, Shape3DBasics
{

   @Override
   default boolean containsNaN()
   {
      return Ellipsoid3DReadOnly.super.containsNaN();
   }
}
