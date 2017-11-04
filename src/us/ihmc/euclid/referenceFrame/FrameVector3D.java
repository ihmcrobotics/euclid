package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code FrameVector3D} is a 3D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector3D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameVector3D} extends {@code Vector3DBasics}, it is compatible with methods
 * only requiring {@code Vector3DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector3D}.
 * </p>
 */
public class FrameVector3D extends FrameTuple3D<FrameVector3D, Vector3D> implements FrameVector3DReadOnly, Vector3DBasics
{
   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector3D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Vector3D());
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new Vector3D(x, y, z));
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      super(referenceFrame, new Vector3D(vectorArray));
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple3DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(referenceFrame, new Vector3D(tuple3DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes its x and y coordinate to {@code tuple2DReadOnly}
    * and to the given reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(referenceFrame, new Vector3D(tuple2DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes its reference frame x and y components from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the components and reference frame from. Not
    *           modified.
    */
   public FrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(frameTuple2DReadOnly.getReferenceFrame(), new Vector3D(frameTuple2DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector3D(FrameTuple3DReadOnly other)
   {
      super(other.getReferenceFrame(), new Vector3D(other));
   }

   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void setAndNormalize(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNormalize(other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code other}.
    * <p>
    * this = this &times; other
    * </p>
    *
    * @param other the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void cross(FrameVector3DReadOnly other)
   {
      cross((FrameTuple3DReadOnly) other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code frameTuple3DReadOnly}.
    * <p>
    * this = this &times; frameTuple3DReadOnly
    * </p>
    *
    * @param frameTuple3DReadOnly the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in
    *            the same reference frame as {@code this}.
    */
   public final void cross(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      tuple.cross(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameVector1} and {@code frameVector2}.
    * <p>
    * this = frameVector1 &times; frameVector2
    * </p>
    *
    * @param frameVector1 the first frame vector in the cross product. Not modified.
    * @param frameVector2 the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameVector1} or {@code frameVector2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   public final void cross(FrameVector3DReadOnly frameVector1, FrameVector3DReadOnly frameVector2)
   {
      cross((FrameTuple3DReadOnly) frameVector1, (FrameTuple3DReadOnly) frameVector2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code frameTuple2}.
    * <p>
    * this = frameTuple1 &times; frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   public final void cross(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.cross(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code tuple2}.
    * <p>
    * this = frameTuple1 &times; tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void cross(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.cross(frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code tuple1} and {@code frameTuple2}.
    * <p>
    * this = tuple1 &times; frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void cross(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.cross(frameTuple1, frameTuple2);
   }

   /**
    * Gets the read-only reference to the vector used in {@code this}.
    *
    * @return the vector of {@code this}.
    */
   public final Vector3D getVector()
   {
      return tuple;
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame vectors
    * are geometrically similar, i.e. the length of the distance between them is
    * less than or equal to {@code epsilon}.
    *
    * @param other the frame vector to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    * @return {@code true} if the two frame vectors represent the same geometry,
    *            {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(FrameVector3D other, double epsilon)
   {
      checkReferenceFrameMatch(other);

      return Vector3DBasics.super.geometricallyEquals(other, epsilon);
   }
}
