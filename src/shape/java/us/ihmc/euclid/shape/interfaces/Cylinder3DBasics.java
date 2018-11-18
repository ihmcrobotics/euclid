package us.ihmc.euclid.shape.interfaces;

public interface Cylinder3DBasics extends Cylinder3DReadOnly, Shape3DBasics
{
   @Override
   default boolean containsNaN()
   {
      return Cylinder3DReadOnly.super.containsNaN();
   }
}
