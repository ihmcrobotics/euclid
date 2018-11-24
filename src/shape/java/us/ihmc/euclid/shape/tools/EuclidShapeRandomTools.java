package us.ihmc.euclid.shape.tools;

import java.util.Collection;
import java.util.Random;

import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.shape.Capsule3D;
import us.ihmc.euclid.shape.Cylinder3D;
import us.ihmc.euclid.shape.Ellipsoid3D;
import us.ihmc.euclid.shape.PointShape3D;
import us.ihmc.euclid.shape.Ramp3D;
import us.ihmc.euclid.shape.Sphere3D;
import us.ihmc.euclid.shape.Torus3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidShapeRandomTools
{
   public static Box3D nextBox3D(Random random)
   {
      return nextBox3D(random, 0.0, 1.0);
   }

   public static Box3D nextBox3D(Random random, double minSize, double maxSize)
   {
      return new Box3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Capsule3D nextCapsule3D(Random random)
   {
      return nextCapsule3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Capsule3D nextCapsule3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Capsule3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                           EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Cylinder3D nextCylinder3D(Random random)
   {
      return nextCylinder3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Cylinder3D nextCylinder3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Cylinder3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                            EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random)
   {
      return nextEllipsoid3D(random, 0.0, 1.0);
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random, double minRadius, double maxRadius)
   {
      return new Ellipsoid3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static PointShape3D nextPointShape3D(Random random)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random));
   }

   public static PointShape3D nextPointShape3D(Random random, double minMax)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   public static Ramp3D nextRamp3D(Random random)
   {
      return nextRamp3D(random, 0.0, 1.0);
   }

   public static Ramp3D nextRamp3D(Random random, double minSize, double maxSize)
   {
      return new Ramp3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Sphere3D nextSphere3D(Random random)
   {
      return nextSphere3D(random, 0.0, 1.0);
   }

   public static Sphere3D nextSphere3D(Random random, double minRadius, double maxRadius)
   {
      return new Sphere3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Torus3D nextTorus3D(Random random)
   {
      return nextTorus3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Torus3D nextTorus3D(Random random, double minRadius, double maxRadius, double minTubeRadius, double maxTubeRadius)
   {
      return new Torus3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                         EuclidCoreRandomTools.nextDouble(random, minTubeRadius, maxTubeRadius));
   }

   public static Point3D nextWeightedAverage(Random random, Collection<? extends Point3DReadOnly> points)
   {
      return nextWeightedAverage(random, points.toArray(new Point3DReadOnly[points.size()]));
   }

   public static Point3D nextWeightedAverage(Random random, Point3DReadOnly[] points)
   {
      double[] weights = new double[points.length];
      double sum = 0.0;

      for (int j = 0; j < weights.length; j++)
         sum += weights[j] = random.nextDouble();

      Point3D next = new Point3D();

      for (int j = 0; j < weights.length; j++)
         next.scaleAdd(weights[j], points[j], next);

      next.scale(1.0 / sum);

      return next;
   }
}
