package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector4D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides random generators to generate random frame geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameRandomTools
{
   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(Random)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(Random random)
   {
      return nextReferenceFrame(random);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random)
   {
      return nextReferenceFrame(random, false);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(Random,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(Random random, boolean use2DTransform)
   {
      return nextReferenceFrame(random, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, boolean use2DTransform)
   {
      return nextReferenceFrame(random, ReferenceFrame.getWorldFrame(), use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(random, parentFrame);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(Random,ReferenceFrame,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return nextReferenceFrame(random, parentFrame, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return nextReferenceFrame("randomFrame" + random.nextInt(), random, parentFrame, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(String,Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(frameName, random, parentFrame);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(frameName, random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    * @deprecated Use {@link #nextReferenceFrame(String,Random,ReferenceFrame,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return nextReferenceFrame(frameName, random, parentFrame, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *           random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      RigidBodyTransform transformFromParent;
      if (use2DTransform)
         transformFromParent = EuclidCoreRandomTools.nextRigidBodyTransform2D(random);
      else
         transformFromParent = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return ReferenceFrame.constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(Random)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random)
   {
      return nextReferenceFrameTree(random);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random)
   {
      return nextReferenceFrameTree(random, false);
   }

   /**
    * Creates a tree structure of 20 random reference frames start off
    * {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(Random,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, boolean use2DTransforms)
   {
      return nextReferenceFrameTree(random, use2DTransforms);
   }

   /**
    * Creates a tree structure of 20 random reference frames start off
    * {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, boolean use2DTransforms)
   {
      return nextReferenceFrameTree(random, 20, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(Random,int)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(random, numberOfReferenceFrames);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(random, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(Random,int,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, int numberOfReferenceFrames, boolean use2DTransforms)
   {
      return nextReferenceFrameTree(random, numberOfReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, int numberOfReferenceFrames, boolean use2DTransforms)
   {
      return nextReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and {@code rootFrame} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(String,Random,ReferenceFrame,int)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfReferenceFrames);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and {@code rootFrame} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and {@code rootFrame} at the first index.
    * @deprecated Use {@link #nextReferenceFrameTree(String,Random,ReferenceFrame,int,boolean)} instead
    */
   @Deprecated
   public static ReferenceFrame[] generateRandomReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames,
         boolean use2DTransforms)
   {
      return nextReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *           random frames.
    * @return the array containing the random reference frame and {@code rootFrame} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames,
         boolean use2DTransforms)
   {
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         referenceFrames[i + 1] = nextReferenceFrame(frameNamePrefix + i, random, parentFrame, use2DTransforms);
      }

      return referenceFrames;
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    * @deprecated Use {@link #nextFramePoint3D(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame)
   {
      return nextFramePoint3D(random, referenceFrame);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-minMax; minMax]. <br>
    * {@code framePoint.y} &in; [-minMax; minMax]. <br>
    * {@code framePoint.z} &in; [-minMax; minMax]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    * @deprecated Use {@link #nextFramePoint3D(Random,ReferenceFrame,double)} instead
    */
   @Deprecated
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return nextFramePoint3D(random, referenceFrame, minMax);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-minMax; minMax]. <br>
    * {@code framePoint.y} &in; [-minMax; minMax]. <br>
    * {@code framePoint.z} &in; [-minMax; minMax]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * {@code framePoint.z} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    * @deprecated Use {@link #nextFramePoint3D(Random,ReferenceFrame,double,double)} instead
    */
   @Deprecated
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return nextFramePoint3D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * {@code framePoint.z} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code framePoint.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code framePoint.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *            {@code maxAbsoluteZ < 0}.
    * @deprecated Use {@link #nextFramePoint3D(Random,ReferenceFrame,double,double,double)} instead
    */
   @Deprecated
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY,
         double maxAbsoluteZ)
   {
      return nextFramePoint3D(random, referenceFrame, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code framePoint.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code framePoint.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *            {@code maxAbsoluteZ < 0}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * {@code framePoint.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @param minZ the minimum value for the z-coordinate.
    * @param maxZ the maximum value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    * @deprecated Use
    *             {@link #nextFramePoint3D(Random,ReferenceFrame,double,double,double,double,double,double)}
    *             instead
    */
   @Deprecated
   public static FramePoint3D generateRandomFramePoint3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY,
         double minZ, double maxZ)
   {
      return nextFramePoint3D(random, referenceFrame, minX, maxX, minY, maxY, minZ, maxZ);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * {@code framePoint.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @param minZ the minimum value for the z-coordinate.
    * @param maxZ the maximum value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY, double minZ,
         double maxZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random frame vector.
    * @deprecated Use {@link #nextFrameVector3D(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameVector3D(random, referenceFrame);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *           frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    * @deprecated Use {@link #nextFrameVector3D(Random,ReferenceFrame,Tuple3DReadOnly)} instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly minMax)
   {
      return nextFrameVector3D(random, referenceFrame, minMax);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *           frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly minMax)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    * @deprecated Use {@link #nextFrameVector3D(Random,ReferenceFrame,Tuple3DReadOnly,Tuple3DReadOnly)}
    *             instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      return nextFrameVector3D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, min, max));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    * @deprecated Use {@link #nextFrameVector3D(Random,ReferenceFrame,double,double)} instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return nextFrameVector3D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, min, max));
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code frameVector.x} &in; [minX; maxX]. <br>
    * {@code frameVector.y} &in; [minY; maxY]. <br>
    * {@code frameVector.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minX the minimum value for the x-component.
    * @param maxX the maximum value for the x-component.
    * @param minY the minimum value for the y-component.
    * @param maxY the maximum value for the y-component.
    * @param minZ the minimum value for the z-component.
    * @param maxZ the maximum value for the z-component.
    * @return the random vector.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    * @deprecated Use
    *             {@link #nextFrameVector3D(Random,ReferenceFrame,double,double,double,double,double,double)}
    *             instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY,
         double minZ, double maxZ)
   {
      return nextFrameVector3D(random, referenceFrame, minX, maxX, minY, maxY, minZ, maxZ);
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code frameVector.x} &in; [minX; maxX]. <br>
    * {@code frameVector.y} &in; [minY; maxY]. <br>
    * {@code frameVector.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minX the minimum value for the x-component.
    * @param maxX the maximum value for the x-component.
    * @param minY the minimum value for the y-component.
    * @param maxY the maximum value for the y-component.
    * @param minZ the minimum value for the z-component.
    * @param maxZ the maximum value for the z-component.
    * @return the random vector.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY, double minZ,
         double maxZ)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    * @deprecated Use {@link #nextFrameVector3DWithFixedLength(Random,ReferenceFrame,double)} instead
    */
   @Deprecated
   public static FrameVector3D generateRandomFrameVector3DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return nextFrameVector3DWithFixedLength(random, referenceFrame, length);
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector3D nextFrameVector3DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param vectorToBeOrthogonalTo the frame vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    * @deprecated Use {@link #nextOrthogonalFrameVector3D(Random,FrameVector3DReadOnly,boolean)}
    *             instead
    */
   @Deprecated
   public static FrameVector3D generateRandomOrthogonalFrameVector3D(Random random, FrameVector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      return nextOrthogonalFrameVector3D(random, vectorToBeOrthogonalTo, normalize);
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param vectorToBeOrthogonalTo the frame vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D nextOrthogonalFrameVector3D(Random random, FrameVector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      return nextOrthogonalFrameVector3D(random, vectorToBeOrthogonalTo.getReferenceFrame(), vectorToBeOrthogonalTo, normalize);
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    * @deprecated Use
    *             {@link #nextOrthogonalFrameVector3D(Random,ReferenceFrame,Vector3DReadOnly,boolean)}
    *             instead
    */
   @Deprecated
   public static FrameVector3D generateRandomOrthogonalFrameVector3D(Random random, ReferenceFrame referenceFrame, Vector3DReadOnly vectorToBeOrthogonalTo,
         boolean normalize)
   {
      return nextOrthogonalFrameVector3D(random, referenceFrame, vectorToBeOrthogonalTo, normalize);
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D nextOrthogonalFrameVector3D(Random random, ReferenceFrame referenceFrame, Vector3DReadOnly vectorToBeOrthogonalTo,
         boolean normalize)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextOrthogonalVector3D(random, vectorToBeOrthogonalTo, normalize));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    * @deprecated Use {@link #nextFramePoint2D(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame)
   {
      return nextFramePoint2D(random, referenceFrame);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-minMax; minMax].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    * @deprecated Use {@link #nextFramePoint2D(Random,ReferenceFrame,double)} instead
    */
   @Deprecated
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return nextFramePoint2D(random, referenceFrame, minMax);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-minMax; minMax].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    * @deprecated Use {@link #nextFramePoint2D(Random,ReferenceFrame,double,double)} instead
    */
   @Deprecated
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return nextFramePoint2D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min the minimum value for each coordinate.
    * @param max the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minX > maxX} or {@code minY > maxY}.
    * @deprecated Use {@link #nextFramePoint2D(Random,ReferenceFrame,double,double,double,double)}
    *             instead
    */
   @Deprecated
   public static FramePoint2D generateRandomFramePoint2D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY)
   {
      return nextFramePoint2D(random, referenceFrame, minX, maxX, minY, maxY);
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX the minimum value for the x-coordinate.
    * @param maxX the maximum value for the x-coordinate.
    * @param minY the minimum value for the y-coordinate.
    * @param maxY the maximum value for the y-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minX > maxX} or {@code minY > maxY}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, minX, maxX, minY, maxY));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame vector.
    * @deprecated Use {@link #nextFrameVector2D(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameVector2D(random, referenceFrame);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    * @deprecated Use {@link #nextFrameVector2D(Random,ReferenceFrame,double,double)} instead
    */
   @Deprecated
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return nextFrameVector2D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min upper-bound for each component of the generated frame vector. Not modified.
    * @param max lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, min, min));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    * @deprecated Use {@link #nextFrameVector2DWithFixedLength(Random,ReferenceFrame,double)} instead
    */
   @Deprecated
   public static FrameVector2D generateRandomFrameVector2DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return nextFrameVector2DWithFixedLength(random, referenceFrame, length);
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param length the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector2D nextFrameVector2DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *           frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    * @deprecated Use {@link #nextFrameVector2D(Random,ReferenceFrame,Tuple2DReadOnly)} instead
    */
   @Deprecated
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly minMax)
   {
      return nextFrameVector2D(random, referenceFrame, minMax);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax tuple used to bound the maximum absolute value of each component of the generated
    *           frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly minMax)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    * @deprecated Use {@link #nextFrameVector2D(Random,ReferenceFrame,Tuple2DReadOnly,Tuple2DReadOnly)}
    *             instead
    */
   @Deprecated
   public static FrameVector2D generateRandomFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      return nextFrameVector2D(random, referenceFrame, min, max);
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min tuple used as upper-bound for each component of the generated frame vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated frame vector. Not
    *           modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, min, max));
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @return the random frame quaternion.
    * @deprecated Use {@link #nextFrameQuaternion(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FrameQuaternion generateRandomFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameQuaternion(random, referenceFrame);
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @return the random frame quaternion.
    */
   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameQuaternion(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random));
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @param minMaxAngle the maximum absolute angle described by the generated quaternion.
    * @return the random frame quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    * @deprecated Use {@link #nextFrameQuaternion(Random,ReferenceFrame,double)} instead
    */
   @Deprecated
   public static FrameQuaternion generateRandomFrameQuaternion(Random random, ReferenceFrame referenceFrame, double minMaxAngle)
   {
      return nextFrameQuaternion(random, referenceFrame, minMaxAngle);
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @param minMaxAngle the maximum absolute angle described by the generated quaternion.
    * @return the random frame quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame referenceFrame, double minMaxAngle)
   {
      return new FrameQuaternion(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random, minMaxAngle));
   }

   /**
    * Generates a random 4D frame vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random 4D frame vector.
    * @deprecated Use {@link #nextFrameVector4D(Random,ReferenceFrame)} instead
    */
   @Deprecated
   public static FrameVector4D generateRandomFrameVector4D(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameVector4D(random, referenceFrame);
   }

   /**
    * Generates a random 4D frame vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random 4D frame vector.
    */
   public static FrameVector4D nextFrameVector4D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector4D(referenceFrame, EuclidCoreRandomTools.nextVector4D(random));
   }

   /**
    * Generates a random 2D frame orientation with a yaw uniformly distributed in [-<i>pi</i>;
    * <i>pi</i>].
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame orientation's reference frame.
    * @return the random 2D frame orientation.
    */
   public static FrameOrientation2D nextFrameOrientation2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameOrientation2D(referenceFrame, EuclidGeometryRandomTools.nextOrientation2D(random));
   }

   /**
    * Generates a random 2D frame pose with a yaw uniformly distributed in [-<i>pi</i>; <i>pi</i>].
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame pose reference frame.
    * @return the random 2D frame pose.
    */
   public static FramePose2D nextFramePose2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePose2D(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random));
   }

   /**
    * Generates a random 2D frame pose.
    * <p>
    * {@code pose.position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].<br>
    * The rotation magnitude described by the orientation part of the generated pose is in
    * [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame pose reference frame.
    * @param positionMinMax the maximum absolute value of each position coordinate.
    * @param orientationMinMax the maximum absolute value of the orientation's magnitude.
    * @return the random 2D frame pose.
    */
   public static FramePose2D nextFramePose2D(Random random, ReferenceFrame referenceFrame, double positionMinMax, double orientationMinMax)
   {
      return new FramePose2D(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random, positionMinMax, orientationMinMax));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * {@code pose.position}<sub>X</sub> &in; [-{@code maxAbsoluteX}; {@code maxAbsoluteX}].<br>
    * {@code pose.position}<sub>Y</sub> &in; [-{@code maxAbsoluteY}; {@code maxAbsoluteY}].<br>
    * {@code pose.position}<sub>Z</sub> &in; [-{@code maxAbsoluteZ}; {@code maxAbsoluteZ}].<br>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @param maxAbsoluteX the maximum absolute value of the position x-coordinate.
    * @param maxAbsoluteY the maximum absolute value of the position y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value of the position z-coordinate.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * {@code pose.position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].<br>
    * The rotation magnitude described by the orientation part of the generated pose is in
    * [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </p>
    * 
    * @param random the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @param positionMinMax the maximum absolute value of each position coordinate.
    * @param orientationMinMax the maximum absolute value of the orientation's magnitude.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame, double positionMinMax, double orientationMinMax)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random, positionMinMax, orientationMinMax));
   }

   /**
    * Generates a random 2D frame line.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame line's reference frame.
    * @return the random frame line.
    */
   public static FrameLine2D nextFrameLine2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLine2D(referenceFrame, EuclidGeometryRandomTools.nextLine2D(random));
   }

   /**
    * Generates a random 2D frame line.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame line's reference frame.
    * @return the random frame line.
    */
   public static FrameLine3D nextFrameLine3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLine3D(referenceFrame, EuclidGeometryRandomTools.nextLine3D(random));
   }

   /**
    * Generates a random 2D frame line segment.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame line segment's reference frame.
    * @return the random frame line segment.
    */
   public static FrameLineSegment2D nextFrameLineSegment2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLineSegment2D(referenceFrame, EuclidGeometryRandomTools.nextLineSegment2D(random));
   }

   /**
    * Generates a random 3D frame line segment.
    *
    * @param random the random generator to use.
    * @param referenceFrame the random frame line segment's reference frame.
    * @return the random frame line segment.
    */
   public static FrameLineSegment3D nextFrameLineSegment3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLineSegment3D(referenceFrame, EuclidGeometryRandomTools.nextLineSegment3D(random));
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    * 
    * @param random the random generator to use.
    * @param referenceFrame the polygon's reference frame.
    * @param maxAbsoluteXY the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *           the random convex polygon. The size of the resulting convex polygon will be less than
    *           {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static FrameConvexPolygon2D nextFrameConvexPolygon2D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      return new FrameConvexPolygon2D(referenceFrame, EuclidGeometryRandomTools.nextConvexPolygon2D(random, maxAbsoluteXY, numberOfPossiblePoints));
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 2D.
    * 
    * @param random the random generator to use.
    * @param referenceFrame the reference frame for the vertices.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static FrameVertex2DSupplier nextFrameVertex2DSupplier(Random random, ReferenceFrame referenceFrame, int numberOfVertices)
   {
      return new FrameVertex2DSupplier()
      {
         Vertex2DSupplier vertex2dSupplier = EuclidGeometryRandomTools.nextVertex2DSupplier(random, numberOfVertices);

         @Override
         public int getNumberOfVertices()
         {
            return vertex2dSupplier.getNumberOfVertices();
         }

         @Override
         public FramePoint2DReadOnly getVertex(int index)
         {
            return new FramePoint2D(referenceFrame, vertex2dSupplier.getVertex(index));
         }
      };
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 3D.
    * 
    * @param random the random generator to use.
    * @param referenceFrame the reference frame for the vertices.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static FrameVertex3DSupplier nextFrameVertex3DSupplier(Random random, ReferenceFrame referenceFrame, int numberOfVertices)
   {
      return new FrameVertex3DSupplier()
      {
         Vertex3DSupplier vertex2dSupplier = EuclidGeometryRandomTools.nextVertex3DSupplier(random, numberOfVertices);

         @Override
         public int getNumberOfVertices()
         {
            return vertex2dSupplier.getNumberOfVertices();
         }

         @Override
         public FramePoint3DReadOnly getVertex(int index)
         {
            return new FramePoint3D(referenceFrame, vertex2dSupplier.getVertex(index));
         }
      };
   }
}
