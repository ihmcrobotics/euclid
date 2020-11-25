package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.SingularOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.junit.jupiter.api.Test;

import javafx.util.Pair;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

public class SingularValueDecomposition3DTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-10;

   @Test
   public void test()
   {
      Random random = new Random(36456);
      SingularValueDecomposition3D svd3d = new SingularValueDecomposition3D();

      long ejmlTotalTime = 0;
      long euclidTotalTime = 0;

      List<Supplier<Pair<Matrix3D, String>>> generators = new ArrayList<>();
      generators.add(() -> new Pair<>(new Matrix3D(RandomMatrices_DDRM.symmetric(3, -100.0, 100.0, random)), "Symmetric matrix"));
      generators.add(() -> new Pair<>(new Matrix3D(EuclidCoreRandomTools.nextRotationMatrix(random, Math.PI)), "Rotation matrix"));
      generators.add(() -> new Pair<>(EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 100.0), "Diagonal matrix"));
      generators.add(() -> new Pair<>(EuclidCoreRandomTools.nextMatrix3D(random, 10.0), "General matrix"));
      generators.add(() -> new Pair<>(EuclidCoreRandomTools.nextMatrix3D(random, 10000.0), "Large values matrix"));

      for (int i = 0; i < 5 * ITERATIONS; i++)
      { // warmup
         new SingularValueDecomposition3D().decompose(EuclidCoreRandomTools.nextMatrix3D(random));
         ejmlSVDDecomposition(EuclidCoreRandomTools.nextMatrix3D(random), new Matrix3D(), new Matrix3D(), new Matrix3D());
      }

      for (Supplier<Pair<Matrix3D, String>> generator : generators)
      {
         for (int i = 0; i < ITERATIONS; i++)
         {
            Matrix3D A = generator.get().getKey();
            long start = System.nanoTime();
            svd3d.decompose(A);
            long end = System.nanoTime();
            euclidTotalTime += end - start;
            double varEpsilon = Math.max(1.0, Math.abs(A.determinant())) * EPSILON;

            Matrix3DReadOnly Ueuclid = new RotationMatrix(svd3d.getU());
            Matrix3DReadOnly Weuclid = svd3d.getW(null);
            Matrix3DReadOnly Veuclid = new RotationMatrix(svd3d.getV());

            assertTrue(Ueuclid.isRotationMatrix(EPSILON));
            assertTrue(Veuclid.isRotationMatrix(EPSILON));

            Matrix3D A_output = new Matrix3D();
            A_output.set(Ueuclid);
            A_output.multiply(Weuclid);
            A_output.multiplyTransposeOther(Veuclid);

            Matrix3D Wejml = new Matrix3D();
            Matrix3D Uejml = new Matrix3D();
            Matrix3D Vejml = new Matrix3D();
            ejmlTotalTime += ejmlSVDDecomposition(A, Uejml, Wejml, Vejml);

            double[] singularValuesEJML = {Wejml.getM00(), Wejml.getM11(), Wejml.getM22()};
            double[] singularValuesEuclid = {Weuclid.getM00(), Weuclid.getM11(), Math.abs(Weuclid.getM22())};

            if (Weuclid.getM22() < 0.0)
            {
               if (columnDot(2, Uejml, Ueuclid) < 0.0)
                  negateColumn(2, Uejml);
               else
                  negateColumn(2, Vejml);
            }

            for (int col = 0; col < 3; col++)
            {
               if (columnDot(col, Uejml, Ueuclid) < 0.0 && columnDot(col, Vejml, Veuclid) < 0.0)
               {
                  negateColumn(col, Uejml);
                  negateColumn(col, Vejml);
               }
            }

            String messagePrefix = "Iteration: " + i + ", generator: " + generator.get().getValue();
            try
            {
               EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, A, A_output, varEpsilon);
               assertArrayEquals(singularValuesEJML, singularValuesEuclid, varEpsilon, messagePrefix);
               if (!EuclidCoreTools.epsilonEquals(singularValuesEJML[0], singularValuesEJML[1], EPSILON)
                     && !EuclidCoreTools.epsilonEquals(singularValuesEJML[0], singularValuesEJML[2], EPSILON))
               { // Can't really compare when singular values are equal, since that pretty much implies an infinite number of solutions.
                  EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, Uejml, Ueuclid, varEpsilon);
                  EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, Vejml, Veuclid, varEpsilon);
               }
            }
            catch (Throwable e)
            {
               System.out.println(messagePrefix);
               System.out.println("A:\n" + A);
               System.out.println("U EJML:\n" + Uejml);
               System.out.println("U Euclid:\n" + Ueuclid);

               System.out.println("W EJML:\n" + Wejml);
               System.out.println("W Euclid:\n" + Weuclid);

               System.out.println("V EJML:\n" + Vejml);
               System.out.println("V Euclid:\n" + Veuclid);

               throw e;
            }
         }
      }

      double euclidAverageMilllis = euclidTotalTime / 1.0e6 / ITERATIONS / generators.size();
      double ejmlAverageMilllis = ejmlTotalTime / 1.0e6 / ITERATIONS / generators.size();
      System.out.println(String.format("Average time in millisec:\n\t-EJML:%s\n\t-Euclid:%s",
                                       Double.toString(ejmlAverageMilllis),
                                       Double.toString(euclidAverageMilllis)));
   }

   static double columnDot(int col, Matrix3DReadOnly a, Matrix3DReadOnly b)
   {
      double dot = 0.0;
      for (int row = 0; row < 3; row++)
      {
         dot += a.getElement(row, col) * b.getElement(row, col);
      }
      return dot;
   }

   static void negateColumn(int col, Matrix3DBasics m)
   {
      for (int row = 0; row < 3; row++)
      {
         m.setElement(row, col, -m.getElement(row, col));
      }
   }

   private static long ejmlSVDDecomposition(Matrix3DReadOnly A, Matrix3DBasics U, Matrix3DBasics W, Matrix3DBasics V)
   {
      DMatrixRMaj A_ejml = new DMatrixRMaj(3, 3);
      A.get(A_ejml);
      SvdImplicitQrDecompose_DDRM svdEJML = new SvdImplicitQrDecompose_DDRM(false, true, true, false);
      long start = System.nanoTime();
      svdEJML.decompose(A_ejml);
      long end = System.nanoTime();
      DMatrixRMaj U_ejml = svdEJML.getU(null, false);
      DMatrixRMaj W_ejml = svdEJML.getW(null);
      DMatrixRMaj V_ejml = svdEJML.getV(null, false);
      SingularOps_DDRM.descendingOrder(U_ejml, false, W_ejml, V_ejml, false);

      U.set(U_ejml);
      W.set(W_ejml);
      V.set(V_ejml);
      return end - start;
   }

   @Test
   public void testSortBColumns()
   {
      Random random = new Random(425346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D diag = EuclidCoreRandomTools.nextDiagonalMatrix3D(random, 10.0);
         Quaternion V = EuclidCoreRandomTools.nextQuaternion(random);

         Matrix3D originalB = new Matrix3D();
         Matrix3DTools.multiply(diag, new RotationMatrix(V), originalB);

         Matrix3D sortedB = new Matrix3D(originalB);
         SingularValueDecomposition3D.sortBColumns(sortedB, V);

         Matrix3D recomputedB = new Matrix3D();
         Matrix3DTools.multiply(diag, new RotationMatrix(V), recomputedB);
         EuclidCoreTestTools.assertMatrix3DEquals(sortedB, recomputedB, EPSILON);
         Vector3D[] cols = {new Vector3D(), new Vector3D(), new Vector3D()};
         sortedB.getColumn(0, cols[0]);
         sortedB.getColumn(1, cols[1]);
         sortedB.getColumn(2, cols[2]);
         assertTrue(cols[0].length() > cols[1].length());
         assertTrue(cols[1].length() > cols[2].length());
      }
   }

   @Test
   public void testSwapColumn()
   {
      Matrix3D original = new Matrix3D(0, 1, 2, 3, 4, 5, 6, 7, 8);

      Matrix3D actual = new Matrix3D();
      Matrix3D expected = new Matrix3D();

      Vector3D tuple1 = new Vector3D();
      Vector3D tuple2 = new Vector3D();

      for (int c1 = 0; c1 < 2; c1++)
      {
         for (int c2 = c1 + 1; c2 < 3; c2++)
         {
            for (boolean negateC1 : new boolean[] {false, true})
            {
               expected.set(original);
               expected.getColumn(c1, tuple1);
               expected.getColumn(c2, tuple2);
               if (negateC1)
                  tuple1.negate();
               expected.setColumn(c1, tuple2);
               expected.setColumn(c2, tuple1);

               actual.set(original);
               swapColumns(c1, negateC1, c2, actual);
               assertEquals(expected, actual);
            }
         }
      }
   }

   @Test
   public void testApplyJacobiGivensRotation()
   {
      Random random = new Random(4576756);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D Q = new Matrix3D();
         Matrix3D S = EuclidCoreRandomTools.nextSymmetricMatrix3D(random, 10.0);

         int p, q;
         Axis3D rotationAxis;
         switch (random.nextInt(3))
         {
            case 0:
               p = 0;
               q = 1;
               rotationAxis = Axis3D.Z;
               break;
            case 1:
               p = 0;
               q = 2;
               rotationAxis = Axis3D.Y;
               break;
            default:
               p = 1;
               q = 2;
               rotationAxis = Axis3D.X;
               break;
         }

         double s_pp = S.getElement(p, p);
         double s_pq = S.getElement(p, q);
         double s_qq = S.getElement(q, q);

         double ch = 2.0 * (s_pp - s_qq);
         double sh = s_pq;

         if (SingularValueDecomposition3D.gamma * sh * sh < ch * ch)
         {
            double omega = 1.0 / Math.sqrt(ch * ch + sh * sh);
            ch *= omega;
            sh *= omega;
         }
         else
         {
            ch = SingularValueDecomposition3D.cosPiOverEight;
            sh = SingularValueDecomposition3D.sinPiOverEight;
         }

         Matrix3D expected = new Matrix3D();
         packGivensRotation(rotationAxis, ch, sh, new Vector4D(), Q);
         expected.setAndTranspose(Q);
         expected.multiply(S);
         expected.multiply(Q);

         Matrix3D actual = new Matrix3D(S);
         applyJacobiGivensRotation(rotationAxis, ch, sh, actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSwapElements()
   {
      Random random = new Random(4677);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int c1, c2;

         switch (random.nextInt(3))
         {
            case 0:
               c1 = 0;
               c2 = 1;
               break;
            case 1:
               c1 = 0;
               c2 = 2;
               break;
            default:
               c1 = 1;
               c2 = 2;
               break;
         }

         Vector4D quaternion = EuclidCoreRandomTools.nextVector4D(random);
         quaternion.normalize();
         Matrix3D original = new Matrix3D();
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), original);

         Matrix3D expected = new Matrix3D();
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), expected);
         swapColumns(c1, true, c2, expected);

         Matrix3D actual = new Matrix3D();
         swapElements(c1, c2, quaternion);
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), actual);
         EuclidCoreTestTools.assertMatrix3DEquals("Iteration: " + i + ", original:\n" + original + "\nc1=" + c1 + ", c2=" + c2, expected, actual, EPSILON);
      }
   }

   static void applyJacobiGivensRotation(Axis3D rotationAxis, double ch, double sh, Matrix3DBasics S)
   {
      switch (rotationAxis)
      {
         case X:
            SingularValueDecomposition3D.applyJacobiGivensRotationX(ch, sh, S);
            break;
         case Y:
            SingularValueDecomposition3D.applyJacobiGivensRotationY(ch, sh, S);
            break;
         case Z:
            SingularValueDecomposition3D.applyJacobiGivensRotationZ(ch, sh, S);
            break;
         default:
            throw new IllegalStateException("Unexpected value for Axis3D: " + rotationAxis);
      }
   }

   static void packGivensRotation(Axis3D rotationAxis, double ch, double sh, Vector4DBasics givensQuaternionToPack, Matrix3DBasics givensRotationToPack)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;

      switch (rotationAxis)
      {
         case X:
            givensQuaternionToPack.set(sh, 0, 0, ch);
            givensRotationToPack.set(1, 0, 0, 0, diag_b, -off_diag, 0, off_diag, diag_b);
            break;
         case Y:
            // Kind off an oddity that the rotation needs to be inverted. 
            // Best guess so far is that sh is computed from the s_02 element which is the location for the +sin(angle) element of a rotation along the y_axis, while the other 2 rotations uses the -sin(angle) element.
            givensQuaternionToPack.set(0, -sh, 0, ch);
            givensRotationToPack.set(diag_b, 0, -off_diag, 0, 1, 0, off_diag, 0, diag_b);
            break;
         case Z:
            givensQuaternionToPack.set(0, 0, sh, ch);
            givensRotationToPack.set(diag_b, -off_diag, 0, off_diag, diag_b, 0, 0, 0, 1);
            break;

         default:
            throw new IllegalStateException("Unexpected value for Axis3D: " + rotationAxis);
      }
   }

   static void swapColumns(int col1, boolean negateCol1, int col2, Matrix3DBasics matrixToSwapColumns)
   {
      if (col2 <= col1)
         throw new IllegalArgumentException("col2 is expected to be strictly greater than col1");

      double r0, r1, r2;
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      if (col1 == 0)
      {
         r0 = matrixToSwapColumns.getM00();
         r1 = matrixToSwapColumns.getM10();
         r2 = matrixToSwapColumns.getM20();

         if (negateCol1)
         {
            r0 = -r0;
            r1 = -r1;
            r2 = -r2;
         }

         if (col2 == 1)
         {
            m00 = matrixToSwapColumns.getM01();
            m01 = r0;
            m02 = matrixToSwapColumns.getM02();
            m10 = matrixToSwapColumns.getM11();
            m11 = r1;
            m12 = matrixToSwapColumns.getM12();
            m20 = matrixToSwapColumns.getM21();
            m21 = r2;
            m22 = matrixToSwapColumns.getM22();
         }
         else // col2 == 2
         {
            m00 = matrixToSwapColumns.getM02();
            m01 = matrixToSwapColumns.getM01();
            m02 = r0;
            m10 = matrixToSwapColumns.getM12();
            m11 = matrixToSwapColumns.getM11();
            m12 = r1;
            m20 = matrixToSwapColumns.getM22();
            m21 = matrixToSwapColumns.getM21();
            m22 = r2;
         }
      }
      else // col1 == 1 & col2 == 2
      {
         r0 = matrixToSwapColumns.getM01();
         r1 = matrixToSwapColumns.getM11();
         r2 = matrixToSwapColumns.getM21();

         if (negateCol1)
         {
            r0 = -r0;
            r1 = -r1;
            r2 = -r2;
         }

         m00 = matrixToSwapColumns.getM00();
         m01 = matrixToSwapColumns.getM02();
         m02 = r0;
         m10 = matrixToSwapColumns.getM10();
         m11 = matrixToSwapColumns.getM12();
         m12 = r1;
         m20 = matrixToSwapColumns.getM20();
         m21 = matrixToSwapColumns.getM22();
         m22 = r2;
      }

      if (matrixToSwapColumns instanceof RotationMatrixBasics)
         ((RotationMatrixBasics) matrixToSwapColumns).setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      else
         matrixToSwapColumns.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   static void swapElements(int c1, int c2, Vector4DBasics quaternion)
   {
      if (c2 <= c1)
         throw new IllegalArgumentException("c2 is expected to be strictly greater than col1");

      double q1x = quaternion.getX() * SingularValueDecomposition3D.sqrtTwoOverTwo;
      double q1y = quaternion.getY() * SingularValueDecomposition3D.sqrtTwoOverTwo;
      double q1z = quaternion.getZ() * SingularValueDecomposition3D.sqrtTwoOverTwo;
      double q1s = quaternion.getS() * SingularValueDecomposition3D.sqrtTwoOverTwo;

      if (c1 == 0)
      {
         if (c2 == 1)
         {
            // QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, 0, 0, sqrtTwoOverTwo, sqrtTwoOverTwo, false, quaternion);
            quaternion.set(q1x + q1y, q1y - q1x, q1s + q1z, q1s - q1z);
         }
         else // c2 == 2
         {
            // QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, 0, -sqrtTwoOverTwo, 0, sqrtTwoOverTwo, false, quaternion);
            quaternion.set(q1x + q1z, q1y - q1s, q1z - q1x, q1s + q1y);
         }
      }
      else // c1 == 1 & c2 == 2
      {
         // QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, sqrtTwoOverTwo, 0, 0, sqrtTwoOverTwo, false, quaternion);
         quaternion.set(q1s + q1x, q1y + q1z, q1z - q1y, q1s - q1x);
      }
   }
}
