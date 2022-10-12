/*----------------------------------------------------------------------------
 * This active-set algorithm attempts to solve quadratic programming problems
 * in the following form:
 *
 * x = arg min   0.5x'Hx + f'x
 * x
 *
 * s.t.  Ax  = b,        linear equality constraints,
 * Lx <= k,        general linear inequality constraints,
 * l  <= x <= u,   bound constraints.
 *
 * Where H is assumed to be positive definite
 *
 * This algorithm is based on Goldfarb and Idnani's [1] dual active-set method,
 * with modifications due to Powell [2].
 *
 * [1] D. Goldfarb and A. Idnani, "A Numerically Stable Dual Method for Solving
 * Strictly Convex Quadratic Programs", Mathematical Programming, vol. 27,
 * pp 1-33, 1983.
 *
 * [2] M.J.D. Powell, "On the Qudratic Programming Algorithm of Goldfrab and
 * Idnani", Mathematical Programming Study, vol. 25, pp 46-61, 1985.
 * --------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
 *
 * +----------------------------------------------+
 * | Written by Adrian Wills,                     |
 * |            School of Elec. Eng. & Comp. Sci. |
 * |            University of Newcastle,          |
 * |            Callaghan, NSW, 2308, AUSTRALIA   |
 * |                                              |
 * | Last Revised  17 October 2008.               |
 * |                                              |
 * | Copyright (C) Adrian Wills.                  |
 * +----------------------------------------------+
 *
 * The current version of this software is free of charge and
 * openly distributed, BUT PLEASE NOTE:
 *
 * This software must be referenced when used in a published work.
 *
 * This software may not be re-distributed as a part of a commercial product.
 * If you distribute it in a non-commercial products, please contact me first,
 * to make sure you ship the most recent version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * IF IT FAILS TO WORK, IT'S YOUR LOSS AND YOUR PROBLEM.
 *
 * -------------------------------------------------------------------------*/
#ifdef INT_INT
#define varint int
#endif

#ifdef INT_LONG_INT
#define varint long int
#endif

#ifdef INT_LONG_LONG_INT
#define varint long long int
#endif

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "qpas_sub_noblas.h"

#define  itlim  40
#define  eps    1e-14


/*-------------------------------------------------------------------------*/
void printvector(const int size, const double * vec, const char * name) {
	int i;
	if (size > 0) {
		printf("\n ");
		printf(name);
		printf(" = \n\n");
		for(i=0;i<size;i++){
			if (vec[i] >= 0.0){printf("          %3.4e\n", vec[i]);}
			else {printf("         %3.4e\n", vec[i]);}
		}
		printf("\n\n");
	}
	else {
		printf("\n ");
		printf(name);
		printf("= []");
	}
}
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
void printmatrix(const int m, const int n, const double * A, const int lda, const char * name) {
	int i, j;
	if (m*n > 0) {
		printf("\n ");
		printf(name);
		printf(" = \n");
		for(i=0;i<m;i++){
			printf("   ");
			for(j=0;j<n;j++){
				if (A[i+j*lda]>0.0){printf("      %3.4e", A[i+j*lda]);}
				else if (A[i+j*lda]<0.0){printf("     %3.4e", A[i+j*lda]);}
				else {printf("      0        ");}
			}
			printf("\n");
		}
		printf("\n\n");
	}
	else {
		printf("\n ");
		printf(name);
		printf("= []");
	}
}
/*-------------------------------------------------------------------------*/

varint qpas_sub_noblas(varint n,
				varint me,
				varint mc,
				varint nl,
				varint nu,
				double * H,
				double * f,
				double * A,
				double * b,
				double * l,
				double * u,
				double * x,
				double * lm,
				varint display,
				varint * numits,
				varint * numadd,
				varint * numdrop) {
	/*-------------------------------------------------------------------------*/
	/* Define variables */
	/*-------------------------------------------------------------------------*/
	double *Z, *R, *y, *r, *rcn, *s, *pt1, *pt2, *pt3, *ptr, *ba;
	double tmp, dtmp, tmp1, dtmp1, tmp2, tmp3, ms, cg, sg, cvmax, xmag, scal, cost, cost_old, cycle_cost;
	double alp, eta, beta, alpha;
	double da, db, cc, ss, rr, zz, rho, scale, fda, fdb, das, dbs, *dx, *dy;
	varint idx, i, j, k, ni, ni1, nna, na, nd, p, maxit, numit, err, ti, itref_count;
	varint mec, kdrop, dropcon, itref, itrfcnt, nk, jn, m, tot_add, tot_drop, xmg, cost_count;
	varint *indx, *indxa, *ptri;
	varint ione  = 1;
	varint izero = 0;
	double one  = 1.0;
	double zero = 0.0;
	char action[15], type[15];
	/*-------------------------------------------------------------------------*/
	
	/*-------------------------------------------------------------------------*/
	/* Allocate memory: this section should be removed later and free memory
	 * should be passed as a pointer */
	/*-------------------------------------------------------------------------*/
	if(me<0){me=0;}if(mc<0){mc=0;}
	mec=me+mc;
	ptr = (double *)malloc(((2*n+6)*n+mec+1)*sizeof(double));
	if(ptr == 0){return -2;}
	i=0; tot_add=0; tot_drop=0;
	Z=&ptr[i];i+=n*n;
	R=&ptr[i];i+=(n+1)*n;
	y=&ptr[i];i+=n+1;
	r=&ptr[i];i+=n;
	rcn=&ptr[i];i+=mec;
	s=&ptr[i];i+=2*n;
	ba=&ptr[i];i+=n;
	
	ptri = (varint *)malloc((3*n+mec+1)*sizeof(varint));
	if(ptri == 0){return -2;}
	i=0;indx=&ptri[i];i+=n+1;indxa=&ptri[i];
	/*-------------------------------------------------------------------------*/
	
	/*-------------------------------------------------------------------------*/
	/* Set some of the initial conditions */
	/*-------------------------------------------------------------------------*/
	na=0; nd=0; maxit=itlim*n; numit=0; itref_count=0; err=-1;
	dropcon=0; xmag=0.0; itref=0; itrfcnt=0; cost_count=0;
	/*-------------------------------------------------------------------------*/
	
	/*-------------------------------------------------------------------------*/
	/* Run some checks on the input variables */
	/*-------------------------------------------------------------------------*/
	/* Check that constraints make sense */
	err=-3; pt1=A;
	for(i=0;i<mec;i++){
		rcn[i]=0.0;
		for(j=0;j<n;j++){rcn[i]+=A[i+j*mec]*A[i+j*mec];}
		if(rcn[i]==0.0){
			/* Equality constraint */
			if(i<me){if(b[i]!=0.0) {
				if(display) {
					printf("\n\nINFO: It looks like the constraints are inconsistent.\n\n");
				}
				err=-7;
				goto exit_stub;}
			}
			/* A(i,:)x=0 can never be <= b(i) if b(i) is < 0 */
			else if(b[i]<0.0) {
				if(display) {
					printf("\n\nINFO: It looks like the constraints are inconsistent.\n\n");
				}
				err=-8;goto exit_stub;
			}
			rcn[i]=0.0;
		}
		else {
			rcn[i]=1/sqrt(rcn[i]);
		}
	}
	/* Do bounds make sense */
	if(nl>0 && nu>0) {
		for(i=0;i<n;i++) {
			if(l[i]>u[i]) {
				if(display) {
					printf("\n\nINFO: It looks like the bounds are inconsistent.\n\n");
				}
				err=-9;goto exit_stub;
			}
		}
	}
	/*-------------------------------------------------------------------------*/
	
	/*-------------------------------------------------------------------------*/
	/* Set the rest of the initial conditions */
	/*-------------------------------------------------------------------------*/
	/* Compute upper triangular Cholesky factor of H*/
	for(i=0;i<n*n;i++){for(j=i;j<n;j++){R[i+j*n] = H[i+j*n];}}
	for(k=0;k<n;k++){
		nk = n*k;
		if (R[nk+k]<=0.0){
			R[nk+k]=1e-15; alpha=1e7;
		} else {
			R[nk+k]=sqrt(R[nk+k]); alpha=1.0/R[nk+k];
		}
		idx = nk+k+n;
		for(i=0;i<n-k-1;i++){
			R[idx]  = alpha*R[idx];
			idx    += n;
		}
		for(j=k+1;j<n;j++){
			alpha    = R[j*n+k];
			for(i=0;i<n-j;i++){
				R[j+(j+i)*n] = R[j+(j+i)*n] - alpha*R[k+(j+i)*n];
			}
		}
	}
	
	/* Form x = -H\f */
	x[0]=-f[0]/R[0];
	for(i=1; i<n;i++){
		x[i]=-f[i];
		for(j=0;j<i;j++){x[i]-=R[j+i*n]*x[j];}
		x[i] = x[i]/R[i+i*n];
	}
	x[n-1]=x[n-1]/R[(n-1)+(n-1)*n];
	for(i=n-2;i>=0;i--){
		for(j=i+1;j<n;j++){x[i]-=R[i+j*n]*x[j];}
		x[i]=x[i]/R[i+i*n];
	}
	
	/* Compute Z=R^-1 */
	Z[0]=1.0/R[0];
	for(i=1;i<n;i++){
		Z[i+i*n]=1.0/R[i+i*n];
		for(j=i-1;j>=0;j--){
			Z[j+i*n] = 0.0;
			for(k=j+1;k<=i;k++){Z[j+i*n]-=R[j+k*n]*Z[n*i+k];}
			Z[j+i*n]=Z[j+i*n]/R[j+j*n];
		}
	}
	
	/* Set initial conditions for Z and indxa */
	for(j=0;j<n-1;j++){k=n*j;for(i=j+1;i<n;i++){Z[k+i]=0.0;}}
	for(i=0;i<mec+2*n;i++){indxa[i]=0;}
	
// 	printvector(n,x,"x");
// 	printmatrix(n,n,Z,n,"Z");
// 	printmatrix(n,n,R,n,"R");
//
// 	goto exit_stub;
	
	/*-------------------------------------------------------------------------*/
	
	/*-------------------------------------------------------------------------*/
	/* Display things . */
	/*-------------------------------------------------------------------------*/
	if(display){
		printf("--------------------------------------------------------------------------------------------------\n");
		printf("%6s%12s%12s%12s%15s%12s%12s%12s\n", "It#", "Cost Diff", "Action", "Cons.#", "Type", "Residual", "Tot. Add", "Tot. Drop");
		printf("--------------------------------------------------------------------------------------------------\n");
	}
	/*-------------------------------------------------------------------------*/
	
// 	printvector(n, x, "x");
	
	/*-------------------------------------------------------------------------*/
	/* Main loop STARTS here. */
	/*-------------------------------------------------------------------------*/
	while(numit<maxit){
		/* Update iteration number */
		numit++;
		cost_count++;
		
		/* if not dropping a constraint then find next most violated */
		if(!dropcon) {
			/* Find next most violated constraint */
			find_next_violated:
				ms=0.0; cvmax=0.0; jn=mec+n; p=-1;
				for(i=0;i<me;i++){
					if(!indxa[i]){
						tmp = b[i];
						for(j=0;j<n;j++){
							tmp -= A[i+mec*j]*x[j];
						}
						tmp1=tmp*rcn[i];
						if(tmp1>0.0){tmp1=-tmp1;}
						if(tmp1<cvmax){cvmax=tmp1; ms=tmp; p=i;}
					}
				}
				if(p==-1){
					for(i=me;i<mec;i++){
						if(!indxa[i]){
							tmp = b[i];
							for(j=0;j<n;j++){
								tmp -= A[i+mec*j]*x[j];
							}
							tmp1=tmp*rcn[i];
							if(tmp1<cvmax){cvmax=tmp1; ms=tmp; p=i;}
						}
					}
					for(i=0;i<nu;i++){
						if(!indxa[mec+i]){
							tmp=u[i]-x[i]; if(tmp<cvmax){cvmax=tmp;ms=tmp;p=mec+i;}
						}
					}
					for(i=0;i<nl;i++){
						if(!indxa[mec+nu+i]){
							tmp=x[i]-l[i]; if(tmp<cvmax){cvmax=tmp;ms=tmp;p=mec+nu+i;}
						}
					}
				}
				/* Check stopping criterion */
				if(cvmax>-eps){
					if(itrfcnt<2){itref=1; goto itref_stub;}
					err=0; goto exit_stub;
				}
				else {
					itrfcnt=0; itref=0;
				}
				
				/* Clear y[na] for Lagrange multiplier let pt1 point to the correct column of R */
				y[na]=0.0; pt1=&R[na*n];
				
				/* Modify r according to type of active constraint */
				if(p<mec){
					for(i=0;i<n;i++){
						pt1[i] = 0.0;
						for(j=0;j<n;j++){
							pt1[i] += Z[j+i*n]*A[p+j*mec];
						}
					}
				}
				else if(p>=mec+nu) {
					for(i=0;i<n;i++){
						pt1[i] = -Z[p-mec-nu+i*n];
					}
				}
				else {
					for(i=0;i<n;i++){
						pt1[i] = Z[p-mec+i*n];
					}
				}
				
// 				printvector(n, pt1, "pt1");
				
				
				/* Do Givens rotation for active constraint. */
				for(i=n-1;i>na;i--) {
					/* Compute and apply Givens matrix */
					da = pt1[i-1];
					db = pt1[i];
					fdb = fabs(db);
					if(fdb>eps) {
						fda = fabs(da);
						rho = db;
						if(fda>fdb){ rho = da;}
						scale = fda+fdb;
						if(scale!=0.0){
							das = da/scale;
							dbs = db/scale;
							rr = scale*sqrt(das*das + dbs*dbs);
							if(rho < 0) { rr = -rr;}
							cc = da/rr;
							ss = db/rr;
							zz = 1.0;
							if(fda>fdb) { zz = ss; }
							if((fdb>=fda) && (cc != 0.0)) {zz = 1.0/cc;}
						} else {
							cc = 1.0;
							ss = 0.0;
							rr = 0.0;
							zz = 0.0;
						}
						pt1[i-1] = rr;
						pt1[i]   = 0.0;
						
						/* Apply Givens rotation matrix */
						dx = &Z[n*(i-1)];
						dy = &Z[n*i];
						for(j=0;j<n;j++){
							da    = cc*dx[j] + ss*dy[j];
							dy[j] = cc*dy[j] - ss*dx[j];
							dx[j] = da;
						}
					}
				}
				
// 				printvector(n, pt1, "pt1");
				
				/* Test to see if currently violated constraint is linearly dependent
				 * NOTE: I have not thought this through very much. */
				if(na>0 && p<me && fabs(R[na+n*na])<eps) {
					/* If it is an equality constraint and the residual is small enough
					 * then just add it to the active set and move on.*/
					if(fabs(ms)<eps) {
						indxa[p]=1;
						goto find_next_violated;
					}
					
					/* Otherwise, we should report that the constraints are linearly dependent */
					if(display) {
						printf("\n\n ERROR: Active equality constraint normals are linearly dependent and residual is %3.5e\n", cvmax);
					}
					err=p+1;
					goto exit_stub;
				}
		}
		
		/* Otherwise, drop the constraint kdrop */
		else{
			/* Shift rows of R */
			for(i=kdrop+1;i<=na;i++){
				for(k=0;k<i+1;k++){R[n*(i-1)+k]=R[n*i+k];}
			}
			if(na==n){j=n-1;}else{j=na;}
			
			/* Do Givens rotation for active cons. */
			for(i=kdrop;i<j;i++){
				/* Compute and apply Givens matrix */
				da = R[i+n*i];
				db = R[i+1+n*i];
				fdb = fabs(db);
				if(fdb>eps) {
					fda = fabs(da);
					rho = db;
					if(fda>fdb){ rho = da;}
					scale = fda+fdb;
					if(scale!=0.0){
						das = da/scale;
						dbs = db/scale;
						rr = scale*sqrt(das*das + dbs*dbs);
						if(rho < 0) { rr = -rr;}
						cc = da/rr;
						ss = db/rr;
						zz = 1.0;
						if(fda>fdb) { zz = ss; }
						if((fdb>=fda) && (cc != 0.0)) {zz = 1.0/cc;}
					} else {
						cc = 1.0;
						ss = 0.0;
						rr = 0.0;
						zz = 0.0;
					}
					R[i+n*i]   = rr;
					R[i+1+n*i] = 0.0;
					
					/* Apply Givens rotation matrix */
					dx = &Z[n*i];
					dy = &Z[n*(i+1)];
					for(k=0;k<n;k++){
						da    = cc*dx[k] + ss*dy[k];
						dy[k] = cc*dy[k] - ss*dx[k];
						dx[k] = da;
					}
					dx = &R[i+n*i];
					dy = &R[i+1+n*i];
					for(k=1;k<na-i;k++){
						da      = cc*dx[n*k] + ss*dy[n*k];
						dy[n*k] = cc*dy[n*k] - ss*dx[n*k];
						dx[n*k] = da;
					}
				}
			}
			
			/* Adjust indecies */
			indxa[indx[kdrop]]=0;
			for(i=kdrop;i<na;i++){indx[i]=indx[i+1];y[i]=y[i+1];}
			nd++; na--;
		}
		
		/* Make sure we don't go beyond n */
		if(na<n){j=na;}else{j=n;}
		
		/* Calculate step to violated constraint  */
		for(i=0;i<n;i++){r[i] = R[n*na+i];}
		
// 		printvector(n, r, "r");
		
		if(na<n){alp  = r[na];} else {alp = r[n-1];}
		eta     = -ms/(alp*alp);
		kdrop   = -1;
		dropcon =  0;
		beta    =  eta;
		if(na>0) {
			/* Calculate change to Lagrange Multipliers */
			r[j-1]=r[j-1]/R[(j-1)+(j-1)*n];
			for(i=j-2;i>=0;i--){
				for(k=i+1;k<j;k++){r[i]-=R[i+k*n]*r[k];}
				r[i]=r[i]/R[i+i*n];
			}
			if(p>=me) {
				for(i=0;i<na;i++) {
					if ((indx[i]>=me) && (r[i]>0.0)) {
						tmp=y[i]/r[i];
						if (((kdrop==-1) && (na==n)) || (tmp<beta)) {
							beta    = tmp;
							kdrop   = i;
							dropcon = 1;
						}
					}
				}
			}
		}
		
// 		printvector(n, r, "r");
		
// 		printf("beta = %3.5e   eta = %3.5e\n", beta, eta);
		//goto exit_stub;
		
		/* Check if ratio is too large meaning constraints are prolly inconsistent */
		if(kdrop>-1) {
			dtmp=1.0/beta;
			if(dtmp<0.0) {
				dtmp=-dtmp;
			}
			if(dtmp<eps) {
				if(display) {
					printf("\n\nINFO: It looks like the constraints are inconsistent.\n\n");
				}
				err=-indx[kdrop];goto exit_stub;
			}
		}
		
		/* Make sure constraints are consistent */
		if(na==n && !dropcon) {
			if(display) {
				printf("\n\nINFO: It looks like the constraints are inconsistent.\n");
				printf("INFO: Since there are %i constraints active, but\n", na);
				printf("INFO: none are being dropped.\n");
			}
			err=-12; goto exit_stub;
		}
		
// 		printvector(n, &Z[n*na], "Z");
		
		/* Take step */
		if(na<n) {
			scal = -alp*beta;
			for(i=0;i<n;i++){x[i] += scal*Z[n*na+i];}
			ms   = ms + beta*alp*alp;
		}
		scal = -beta;
		for(i=0;i<j;i++){y[i] += scal*r[i];}
		y[na] = y[na] + beta;
		
// 		printvector(n, y, "y");
		
		/* Add constraint to active set if not dropping one */
		if(!dropcon) {
			/* Add constraint */
			indx[na]=p; indxa[p]=1; na++;
			
// 			/* Test first order conditions */
// 			dcopy(&n, f, &ione, r, &ione);
// 			dsymv("U", &n, &one, H, &n, x, &ione, &one, r, &ione);
// 			for(i=0;i<na;i++) {
// 				if(indx[i]<mec) {
// 					daxpy(&n, &y[i], &A[indx[i]], &mec, r, &ione);
// 				}
// 				else if(indx[i]<mec+nu) {
// 					r[indx[i]-mec]+=y[i];
// 				}
// 				else {
// 					r[indx[i]-mec-nu]-=y[i];
// 				}
// 			}
// 			printvector(n, r, "res");
			
			/* Calculate the magnitude of x and reinitialise if necessary */
			tmp=0.0;for(i=0;i<n;i++){tmp+=fabs(x[i])*(fabs(f[i]) + fabs(H[(n+1)*i]*x[i]));}
			if(xmag<tmp){xmag=tmp;}
			
			/* Iterative refinement code - also does reinitialisation if xmag too big */
			if((tmp<=0.01*xmag)){
				
				itref_stub:
					if(!itref){for(i=0;i<n;i++){x[i]=0.0; y[i]=0.0;}}
					pt1=r;
					for(i=0;i<n;i++){r[i]=f[i];}
					if(itref){
						for(i=0;i<n;i++){
							for(k=0;k<n;k++){
								r[i] += H[i+n*k]*x[k];
							}
						}
					}
					for(i=0;i<na;i++){
						if(indx[i]<mec){
							s[n+i]=-b[indx[i]];
							ba[i] = b[indx[i]];
							if(itref){
								for(k=0;k<n;k++){
									r[k] += A[indx[i]+mec*k]*y[i];
									s[n+i] += A[indx[i]+mec*k]*x[k];
								}
							}
						}
						else if(indx[i]<mec+nu){
							s[n+i]=-u[indx[i]-mec];
							ba[i] = u[indx[i]-mec];
							if(itref){
								pt1[indx[i]-mec]+=y[i];
								s[n+i]+=x[indx[i]-mec];
							}
						}
						else{
							s[n+i]=  l[indx[i]-mec-nu];
							ba[i] = -l[indx[i]-mec-nu];
							if(itref){
								pt1[indx[i]-mec-nu]-=y[i];
								s[n+i]-=x[indx[i]-mec-nu];
							}
						}
					}
					s[n]=s[n]/R[0];
					for(i=1; i<na;i++){
						for(j=0;j<i;j++){s[n+i]-=R[j+i*n]*s[n+j];}
						s[n+i] = s[n+i]/R[i+i*n];
					}
					if(na==0){for(i=0;i<n;i++){s[i]=0.0;}}
					else{
						for(i=0;i<n;i++){
							s[i] = 0.0;
							for(k=0;k<na;k++){
								s[i] -= Z[i+k*n]*s[n+k];
							}
						}
						for(i=0;i<n;i++){x[i]+=s[i];}
						for(i=0;i<n;i++){
							for(k=0;k<n;k++){
								r[i] += H[i+k*n]*s[k];
							}
						}
						for(i=0;i<n;i++){
							s[i]=0.0;
							for(k=0;k<n;k++){
								s[i] += Z[k+i*n]*r[k];
							}
						}
						for(i=0;i<n;i++){
							for(k=0;k<n-na;k++){
								x[i] -= Z[n*na+i+k*n]*s[na+k];
							}
						}
						s[na-1]=s[na-1]/R[(na-1)+(na-1)*n];
						for(i=na-2;i>=0;i--){
							for(j=i+1;j<na;j++){s[i]-=R[i+j*n]*s[j];}
							s[i]=s[i]/R[i+i*n];
						}
						for(i=0;i<na;i++){y[i]-=s[i];}
					}
					
					/* Check Lagrange Multipliers */
					tmp=1.0;for(i=0;i<na;i++){if(indx[i]>=me && y[i]<tmp){kdrop=i;tmp=y[i];}}
					if(tmp<-eps){if(display){printf("WARNING: Neg. Lagrange Mult: %3.5g\n", tmp); dropcon=1; itref=0; itrfcnt=0; err=13; goto exit_stub;}}
					else{if(itref){itrfcnt++;}}
					if(!itref){xmg=1;}
					xmag=0.0;
			}
		}
		
		/* Calculate cost */
		for(i=0;i<n;i++){s[i]=2.0*f[i];}
		cost = 0.0;
		for(i=0;i<n;i++){
			for(k=0;k<n;k++){
				s[i] += H[i+k*n]*x[k];
			}
			cost += s[i]*x[i];
		}
		if(cost_count==1) {
			cycle_cost=cost;
		}
		else if(cost_count>=3) {
			if(fabs(cost-cycle_cost)<eps) {
				if(display) {
					printf("\n\nWARNING: Cannot increase cost, exiting early and result may be dubious.\n");
				}
				err=0;
				goto exit_stub;
			}
			cycle_cost=cost;
			cost_count=0;
		}
		
		
		/* Report info to users */
		if(itref){
			strcpy(action, "refine");
			strcpy(type, "n/a");
			i=-1;
		}
		else{
			if(xmg){strcpy(action, "add+ref"); i=p; tot_add+=1;}
			else if(dropcon){strcpy(action, "drop"); i=indx[kdrop]; tot_drop+=1;}
			else {strcpy(action, "add"); i=p; tot_add+=1;}
			if(i<me){strcpy(type, "equality");}
			else if(i<mec){strcpy(type, "inequality");}
			else if(i<mec+nl){strcpy(type, "lower bound");}
			else if(i<mec+nl+nu){strcpy(type, "upper bound");}
			else{strcpy(type, "error");}
		}
		if(display){
			if(numit>1) {
				printf("%6i%12.2e%12s%12i%15s%12.2e%12i%12i\n", numit, cost-cost_old, action, i+1, type, cvmax, tot_add, tot_drop);
			}
			else {
				printf("%6i%12s%12s%12i%15s%12.2e%12i%12i\n", numit, "N/A", action, i+1, type, cvmax, tot_add, tot_drop);
			}
		}
		itref=0; xmg=0;
		cost_old = cost;
	}
	
	/*If we make it here then too many iterations have passed */
	err=-1;
	if(display) {
		printf("\n\nINFO: Maximum number of iterations exceeded.\n\n");
	}
	/*-------------------------------------------------------------------------*/
	/* Main loop ENDS here*/
	/*-------------------------------------------------------------------------*/
	
	exit_stub:
		/*Update numits counter for return*/
		numits[0]  = numit;
		numadd[0]  = tot_add;
		numdrop[0] = tot_drop;
		
		/* Check for errors */
		if(err>=0){
			/* Make sure final x is inside the box */
			for(i=0;i<nl;i++){if(x[i]<l[i]){x[i]=l[i];}}
			for(i=0;i<nu;i++){if(x[i]>u[i]){x[i]=u[i];}}
		}
		
		/* Copy Lagrange multipliers into lm */
		for(i=0;i<na;i++){
			lm[indx[i]]=y[i];
		}
		
		/* De-allocate memory */
		free(ptr);
		free(ptri);
		
		if(display && err!=0) {
			printf("\nERROR: There was an ERROR in attemping to solve this problem.\n\n");
		}
		else {
			if(display) {
				printf("--------------------------------------------------------------------------------------------------\n");
			}
		}
		return err;
}
/*-------------------------------------------------------------------------*/

#undef eps
#undef itlim
