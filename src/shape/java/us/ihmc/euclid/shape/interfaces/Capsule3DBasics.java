package us.ihmc.euclid.shape.interfaces;

public interface Capsule3DBasics extends Capsule3DReadOnly, Shape3DBasics
{
   @Override
   default boolean containsNaN()
   {
      return Capsule3DReadOnly.super.containsNaN();
   }
}
