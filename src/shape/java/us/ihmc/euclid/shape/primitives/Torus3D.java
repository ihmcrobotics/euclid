package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a torus 3D.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Torus3D implements Torus3DBasics
{
   /** Position of this torus' center. */
   private final Point3D position = new Point3D();
   /** Axis of revolution of this torus. */
   private final UnitVector3D axis = new UnitVector3D(Axis3D.Z);

   /** It is the radius for the center of the torus to the center of the tube. */
   private double radius;
   /** Represents the radius of the tube */
   private double tubeRadius;

   /**
    * Creates a new torus 3D with a radius of {@code 1}, and tube radius of {@code 0.1} and which axis
    * is along the z-axis.
    */
   public Torus3D()
   {
      this(1.0, 0.1);
   }

   /**
    * Creates a new torus 3D and initializes its radii.
    *
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code radius} or {@code tubeRadius} is negative.
    */
   public Torus3D(double radius, double tubeRadius)
   {
      setRadii(radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and radii.
    *
    * @param position   the position of the center. Not modified.
    * @param axis       the axis of revolution. Not modified.
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code radius} or {@code tubeRadius} is negative.
    */
   public Torus3D(Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      set(position, axis, radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D identical to {@code other}.
    *
    * @param other the other torus to copy. Not modified.
    */
   public Torus3D(Torus3DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets the radii of this torus 3D.
    *
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code radius} or {@code tubeRadius} is negative.
    */
   @Override
   public void setRadii(double radius, double tubeRadius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a " + getClass().getSimpleName() + " cannot be negative: " + radius);
      if (tubeRadius < 0.0)
         throw new IllegalArgumentException("The tube radius of a " + getClass().getSimpleName() + " cannot be negative: " + tubeRadius);

      this.radius = radius;
      this.tubeRadius = tubeRadius;
   }

   /** {@inheritDoc} */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /** {@inheritDoc} */
   @Override
   public double getTubeRadius()
   {
      return tubeRadius;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public UnitVector3DBasics getAxis()
   {
      return axis;
   }

   @Override
   public Torus3D copy()
   {
      return new Torus3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Torus3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Torus3DReadOnly)
         return Torus3DBasics.super.equals((Torus3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this torus 3D.
    *
    * @return the hash code value for this torus 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = 1L;
      hash = EuclidHashCodeTools.addToHashCode(hash, radius);
      hash = EuclidHashCodeTools.addToHashCode(hash, tubeRadius);
      hash = EuclidHashCodeTools.combineHashCode(hash, position.hashCode());
      hash = EuclidHashCodeTools.combineHashCode(hash, axis.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this torus 3D as follows:<br>
    * Torus 3D: [position: (-0.362, -0.617, 0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius: 0.170,
    * tube radius: 0.906]
    *
    * @return the {@code String} representing this torus 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
