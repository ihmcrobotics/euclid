package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FrameVector2D} is a 2D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector2D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameVector2D} extends {@code Vector2DBasics}, it is compatible with methods
 * only requiring {@code Vector2DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector2D}.
 * </p>
 */
public class FrameVector2D implements FrameVector2DBasics, GeometryObject<FrameVector2D>
{
   /** The reference frame is which this vector is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The vector holding the current components of this frame vector. */
   private final Vector2D vector = new Vector2D();
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to the
    * {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given reference
    * frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, double x, double y)
   {
      setIncludingFrame(referenceFrame, x, y);
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y} in order from the
    * given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      setIncludingFrame(referenceFrame, vectorArray);
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple2DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple2DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new frame vector and initializes it to the x and y coordinates of
    * {@code tuple3DReadOnly} and to the given reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FrameVector2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector2D(FrameTuple2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new frame vector and initializes it to x and y components of
    * {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the tuple to copy the components and reference frame from. Not
    *           modified.
    */
   public FrameVector2D(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      setIncludingFrame(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame vector to {@code other}.
    *
    * @param other the other frame vector to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(FrameVector2D other)
   {
      FrameVector2DBasics.super.set(other);
   }

   /**
    * Sets the reference frame of this vector without updating or modifying its x and y components.
    *
    * @param referenceFrame the new reference frame for this frame vector.
    */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the x-component of this vector.
    *
    * @param x the x-component.
    */
   @Override
   public void setX(double x)
   {
      vector.setX(x);
   }

   /**
    * Sets the y-component of this vector.
    *
    * @param y the y-component.
    */
   @Override
   public void setY(double y)
   {
      vector.setY(y);
   }

   /**
    * Gets the reference frame in which this vector is currently expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Returns the value of the x-component of this vector.
    *
    * @return the x-component's value.
    */
   @Override
   public double getX()
   {
      return vector.getX();
   }

   /**
    * Returns the value of the y-component of this vector.
    *
    * @return the y-component's value.
    */
   @Override
   public double getY()
   {
      return vector.getY();
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of checking
       * both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple2DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two vectors are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameTuple2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameVector2D other, double epsilon)
   {
      return FrameVector2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same vector 2D to an {@code epsilon}.
    * <p>
    * Two vectors are considered geometrically equal if they are at a distance of less than or equal to
    * {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other vector 2D to compare against this. Not modified.
    * @param epsilon the maximum distance that the two vectors can be spaced and still considered
    *           equal.
    * @return {@code true} if the two vectors represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FrameVector2D other, double epsilon)
   {
      return FrameVector2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame vector 2D as follows: (x, y)-worldFrame.
    *
    * @return the {@code String} representing this frame vector 2D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame vector
    * 2D.
    *
    * @return the hash code value for this frame vector 2D.
    */
   @Override
   public int hashCode()
   {
      return vector.hashCode();
   }
}
