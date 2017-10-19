package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * {@code FrameQuaternion} is a quaternion expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link QuaternionBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameQuaternion}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a vector in
 * different reference frame.
 * </p>
 * <p>
 * Because a {@code FrameQuaternion} extends {@code QuaternionBasics}, it is compatible with methods
 * only requiring {@code QuaternionBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameQuaternion}.
 * </p>
 */
public class FrameQuaternion extends FrameTuple4D<FrameQuaternion, Quaternion> implements FrameQuaternionReadOnly, QuaternionBasics
{
   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameQuaternion()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Quaternion());
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
   public FrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      super(referenceFrame, new Quaternion(x, y, z, s));
   }

   /**
    * Creates a new frame quaternion and initializes its component {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this quaternion's components. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      super(referenceFrame, new Quaternion(vectorArray));
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code tuple4DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple4DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      super(referenceFrame, new Quaternion(tuple4DReadOnly));
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameQuaternion(FrameQuaternionReadOnly other)
   {
      super(other.getReferenceFrame(), new Quaternion(other));
   }

   /**
    * Sets this frame quaternion to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void setAndNormalize(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNormalize(other);
   }

   /**
    * Gets the read-only reference to the quaternion used in {@code this}.
    *
    * @return the quaternion of {@code this}.
    */
   public final QuaternionReadOnly getQuaternion()
   {
      return tuple;
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      tuple.setUnsafe(qx, qy, qz, qs);
   }
}
