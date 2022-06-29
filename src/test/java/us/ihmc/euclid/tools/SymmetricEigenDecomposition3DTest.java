package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SymmetricEigenDecomposition3DTest
{
   private static final boolean VERBOSE = false;
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void test()
   {
      Random random = new Random(7634534);
      SymmetricEigenDecomposition3D eigenEuclid = new SymmetricEigenDecomposition3D();

      long ejmlTotalTime = 0;
      long euclidTotalTime = 0;

      for (int i = 0; i < 5 * ITERATIONS; i++)
      { // warmup
         new SymmetricEigenDecomposition3D().decompose(EuclidCoreRandomTools.nextSymmetricMatrix3D(random));
         ejmlEigenDecomposition(EuclidCoreRandomTools.nextSymmetricMatrix3D(random), new Matrix3D(), new Vector3D());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Matrix3D A = EuclidCoreRandomTools.nextSymmetricMatrix3D(random, 5.0);
         long start = System.nanoTime();
         eigenEuclid.decompose(A);
         long end = System.nanoTime();
         euclidTotalTime += end - start;
         double varEpsilon = Math.max(1.0, Math.abs(A.determinant())) * EPSILON;

         Matrix3DBasics Qeuclid = eigenEuclid.getEigenVectors(null);
         Vector3DBasics lambdaeuclid = eigenEuclid.getEigenValues();

         Matrix3D Qejml = new Matrix3D();
         Vector3D lambdaejml = new Vector3D();
         ejmlTotalTime += ejmlEigenDecomposition(A, Qejml, lambdaejml);

         performAssertions(i, A, Qeuclid, lambdaeuclid, Qejml, lambdaejml, varEpsilon);
      }

      double euclidAverageMilllis = euclidTotalTime / 1.0e6 / ITERATIONS;
      double ejmlAverageMilllis = ejmlTotalTime / 1.0e6 / ITERATIONS;
      if (VERBOSE)
         System.out.println(String.format("Average time in millisec:\n\t-EJML:%s\n\t-Euclid:%s",
                                          Double.toString(ejmlAverageMilllis),
                                          Double.toString(euclidAverageMilllis)));
   }

   @Test
   public void testBug1()
   {
      Matrix3D A = new Matrix3D(0.2808695734177750000,
                                0.2220959555782587600,
                                -0.2061624511530457700,
                                0.2220959555782587600,
                                0.4569467556886334000,
                                0.1400465337625852200,
                                -0.2061624511530457700,
                                0.1400465337625852200,
                                0.4778175677529285500);
      SymmetricEigenDecomposition3D eigenEuclid = new SymmetricEigenDecomposition3D();
      assertTrue(eigenEuclid.decompose(A));
      double varEpsilon = Math.max(1.0, Math.abs(A.determinant())) * EPSILON;

      Matrix3DBasics Qeuclid = eigenEuclid.getEigenVectors(null);
      Vector3DBasics lambdaeuclid = eigenEuclid.getEigenValues();

      Matrix3D Qejml = new Matrix3D();
      Vector3D lambdaejml = new Vector3D();
      ejmlEigenDecomposition(A, Qejml, lambdaejml);
      performAssertions(0, A, Qeuclid, lambdaeuclid, Qejml, lambdaejml, varEpsilon);
   }

   @Test
   public void testBug2()
   {
      Matrix3D A = new Matrix3D(1.8570857023973861E-6,
                                -5.783382514121337E-6,
                                0.0,
                                -5.783382514121337E-6,
                                1.801075376406469E-5,
                                0.0,
                                0.0,
                                0.0,
                                0.003445569794329082);
      SymmetricEigenDecomposition3D eigenEuclid = new SymmetricEigenDecomposition3D();
      assertTrue(eigenEuclid.decompose(A));

      Matrix3DBasics Qeuclid = eigenEuclid.getEigenVectors(null);
      Vector3DBasics lambdaeuclid = eigenEuclid.getEigenValues();

      Matrix3D Qejml = new Matrix3D();
      Vector3D lambdaejml = new Vector3D();
      ejmlEigenDecomposition(A, Qejml, lambdaejml);
      performAssertions(0, A, Qeuclid, lambdaeuclid, Qejml, lambdaejml, EPSILON);
   }

   private void performAssertions(int i,
                                  Matrix3DReadOnly A,
                                  Matrix3DBasics Qeuclid,
                                  Vector3DBasics lambdaeuclid,
                                  Matrix3DBasics Qejml,
                                  Vector3DBasics lambdaejml,
                                  double epsilon)
   {
      for (int col = 0; col < 3; col++)
      {
         if (SingularValueDecomposition3DTest.columnDot(col, Qejml, Qeuclid) < 0.0)
         {
            SingularValueDecomposition3DTest.negateColumn(col, Qejml);
         }
      }

      Matrix3D A_output = new Matrix3D();
      A_output.set(Qeuclid);
      A_output.multiply(new Matrix3D(lambdaeuclid.getX(), 0, 0, 0, lambdaeuclid.getY(), 0, 0, 0, lambdaeuclid.getZ()));
      A_output.multiplyTransposeOther(Qeuclid);

      String messagePrefix = "Iteration: " + i;

      EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, A, A_output, epsilon);
      EuclidCoreTestTools.assertEquals(messagePrefix, lambdaejml, lambdaeuclid, epsilon);
      if (EuclidCoreTools.epsilonEquals(lambdaejml.getX(), lambdaejml.getY(), epsilon))
      {
         if (!EuclidCoreTools.epsilonEquals(lambdaejml.getY(), lambdaejml.getZ(), epsilon))
         {
            Vector3D zColumnEJML = new Vector3D();
            Vector3D zColumnEuclid = new Vector3D();
            Qejml.getColumn(2, zColumnEJML);
            Qeuclid.getColumn(2, zColumnEuclid);
            EuclidCoreTestTools.assertEquals(zColumnEJML, zColumnEuclid, epsilon);
         }
      }
      else
      {
         if (!EuclidCoreTools.epsilonEquals(lambdaejml.getY(), lambdaejml.getZ(), epsilon))
         {
            EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix, Qejml, Qeuclid, epsilon);
         }
         else
         {
            Vector3D xColumnEJML = new Vector3D();
            Vector3D xColumnEuclid = new Vector3D();
            Qejml.getColumn(0, xColumnEJML);
            Qeuclid.getColumn(0, xColumnEuclid);
            EuclidCoreTestTools.assertEquals(xColumnEJML, xColumnEuclid, epsilon);
         }
      }
   }

   private static long ejmlEigenDecomposition(Matrix3DReadOnly A, Matrix3DBasics Q, Vector3DBasics lambda)
   {
      DMatrixRMaj A_ejml = new DMatrixRMaj(3, 3);
      A.get(A_ejml);
      EigenDecomposition_F64<DMatrixRMaj> eigenEJML = DecompositionFactory_DDRM.eig(3, true, true);
      long start = System.nanoTime();
      eigenEJML.decompose(A_ejml);
      long end = System.nanoTime();

      int[] order = {0, 1, 2};
      double rho0 = eigenEJML.getEigenvalue(0).getReal();
      double rho1 = eigenEJML.getEigenvalue(1).getReal();
      double rho2 = eigenEJML.getEigenvalue(2).getReal();

      if (Math.abs(rho0) < Math.abs(rho1))
      {
         int tempInt = order[0];
         order[0] = order[1];
         order[1] = tempInt;

         double tempDouble = rho0;
         rho0 = rho1;
         rho1 = tempDouble;
      }

      if (Math.abs(rho0) < Math.abs(rho2))
      {
         int tempInt = order[0];
         order[0] = order[2];
         order[2] = tempInt;

         double tempDouble = rho0;
         rho0 = rho2;
         rho2 = tempDouble;
      }

      if (Math.abs(rho1) < Math.abs(rho2))
      {
         int tempInt = order[1];
         order[1] = order[2];
         order[2] = tempInt;
      }

      int col = 0;

      for (int i : order)
      {
         lambda.setElement(col, eigenEJML.getEigenvalue(i).getReal());
         double x = eigenEJML.getEigenVector(i).get(0);
         double y = eigenEJML.getEigenVector(i).get(1);
         double z = eigenEJML.getEigenVector(i).get(2);
         Q.setColumn(col, x, y, z);
         col++;
      }

      return end - start;
   }
}
