package us.ihmc.euclid.shape.interfaces;

public interface Cylinder3DBasics extends Cylinder3DReadOnly, Shape3DBasics
{
   /**
    * Sets the radius of this cylinder.
    *
    * @param radius the new radius for this cylinder.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   void setRadius(double radius);

   /**
    * Sets the height of this cylinder.
    *
    * @param height the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code height} is negative.
    */
   void setHeight(double height);

   @Override
   default boolean containsNaN()
   {
      return Cylinder3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      Shape3DBasics.super.setToNaN();
      setHeight(Double.NaN);
      setRadius(Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
      setHeight(0.0);
      setRadius(0.0);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   default void set(Cylinder3DReadOnly other)
   {
      setPose(other);
      setHeight(other.getHeight());
      setRadius(other.getRadius());
   }

}
