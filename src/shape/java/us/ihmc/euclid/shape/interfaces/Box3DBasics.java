package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Box3DBasics extends Box3DReadOnly, Shape3DBasics
{
   Vector3DBasics getSize();

   @Override
   default boolean containsNaN()
   {
      return Box3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      Shape3DBasics.super.setToNaN();
      getSize().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
      getSize().setToZero();
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param other the other box to copy. Not modified.
    */
   default void set(Box3DReadOnly other)
   {
      setPose(other);
      getSize().set(other.getSize());
   }

   /**
    * Sets the size of this box.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of the three arguments is negative.
    */
   default void setSize(double sizeX, double sizeY, double sizeZ)
   {
      if (sizeX < 0.0 || sizeY < 0.0 || sizeZ < 0.0)
         throw new IllegalArgumentException("A Box3D cannot have a negative size: " + EuclidCoreIOTools.getStringOf("(", ")", ",", sizeX, sizeY, sizeZ));

      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Applies the given scale factor to the size of this box.
    *
    * @param scale the scale factor to use.
    * @throws IllegalArgumentException if {@code scale} is negative.
    */
   default void scale(double scale)
   {
      if (scale < 0.0)
         throw new IllegalArgumentException("Cannot apply a negative scale: " + scale);
      getSize().scale(scale);
   }

}
