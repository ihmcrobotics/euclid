package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTorus3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTorus3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a torus 3D expressed in a given reference frame.
 * <p>
 * A torus 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameTorus3D implements FrameTorus3DBasics
{
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** Position of this torus's center. */
   private final FixedFramePoint3DBasics position = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** Axis of revolution of this torus. */
   private final FixedFrameUnitVector3DBasics axis = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(this, Axis3D.Z);
   /** It is the radius for the center of the torus to the center of the tube. */
   private double radius;
   /** Represents the radius of the tube */
   private double tubeRadius;

   /**
    * Creates a new torus which axis is along the z-axis, a radius of 1, and tube radius of 0.1 and
    * initializes its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameTorus3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new torus which axis is along the z-axis, a length of 1, and radius of 0.5 and
    * initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameTorus3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 0.5);
   }

   /**
    * Creates a new torus which axis is along the z-axis and initializes its size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException if {@code radius} or {@code tubeRadius} is negative.
    */
   public FrameTorus3D(ReferenceFrame referenceFrame, double radius, double tubeRadius)
   {
      setReferenceFrame(referenceFrame);
      setRadii(radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of the center. Not modified.
    * @param axis           the axis of revolution. Not modified.
    * @param radius         radius from the torus center to the tube center.
    * @param tubeRadius     radius of the torus' tube.
    * @throws IllegalArgumentException if {@code radius} or {@code tubeRadius} is negative.
    */
   public FrameTorus3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      setIncludingFrame(referenceFrame, position, axis, radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and size.
    *
    * @param position   the position of the center. Not modified.
    * @param axis       the axis of revolution. Not modified.
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException        if {@code radius} or {@code tubeRadius} is negative.
    * @throws ReferenceFrameMismatchException if the frame argument are not expressed in the same
    *                                         reference frame.
    */
   public FrameTorus3D(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double radius, double tubeRadius)
   {
      setIncludingFrame(position, axis, radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D identical to {@code other}.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other torus to copy. Not modified.
    */
   public FrameTorus3D(ReferenceFrame referenceFrame, Torus3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   /**
    * Creates a new torus 3D identical to {@code other}.
    *
    * @param other the other torus to copy. Not modified.
    */
   public FrameTorus3D(FrameTorus3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameUnitVector3DBasics getAxis()
   {
      return axis;
   }

   /** {@inheritDoc} */
   @Override
   public FrameTorus3D copy()
   {
      return new FrameTorus3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTorus3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two tori have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTorus3DReadOnly)
         return FrameTorus3DBasics.super.equals((FrameTorus3DReadOnly) object);
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
    * Provides a {@code String} representation of this torus 3D as follows:
    *
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this torus 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
