# ephemerand

global randomness beacon

## Motivation

In decentralised systems there is often a need for random numbers. Despite the system participants not trusting one another, the following properties are required:

* **Unpredictable**: Nobody should be able to predict the number in advance.
* **Unbiasable**: Nobody should be able to influence number.

## Survey of approaches

### Block-hashes

In a blockchain system such as Bitcoin, collections of transactions are grouped into blocks and then hashed. Since these blocks are replicated across the world, they can be used as randomness beacons, however there are [many caveats to be aware of](https://blog.positive.com/predicting-random-numbers-in-ethereum-smart-contracts-e5358c6b8620). For instance, the miner who constructs the block can influence the randomness by choosing to not publish the block if the blockhash is disadvantageous. In non-Proof-of-Work blockchains this problem is even worse, since a miner may be able to rapidly construct many valid blocks and suggest the one that has the most advantageous hash.

### Commit-reveal

Another approach is to have all the participants who have an interest in the random value to submit a hash of their own personally generated random number. After everybody has comitted to their hashes, they then reveal their personally generated random numbers. Every participant verifies everyone else's comitted hashes were computed properly, and then combines the random values into a final random number. Similar to block-hashes, the last participant to reveal can choose to not reveal if the final random number is disadvantageous.

### Variable Delay Functions

A [https://vdfresearch.org/](Variable Delay Function) is a function that takes a very long time to compute and cannot be computed in parallel. The input from one of the above schemes can be fed into a VDF, and people can commit to being bound by the final result of the VDF before it is possible that anybody could compute it. Then somebody goes ahead working on computing the VDF to produce the final random value. Ideally the VDF can be verified to be computed correctly in less time than it took to compute it. Most currently-known VDFs require trusted setups, and tuning the delay parameter requires careful tuning (possibly needing specialized hardware as a benchmark).

### NIST randomness beacon

The [NIST randomness beacon](https://www.nist.gov/programs-projects/nist-randomness-beacon) is a system that periodically broadcasts randomly-generated values over the internet, signing with NIST's private key to ensure it hasn't been tampered with. Of course to rely on this you need to trust that NIST is generating the random values honestly.

### Real-world values

The whole world is full of random numbers. For example, you could flip a coin a bunch of times. The challenge is proving to everyone else that somebody has honestly recorded them, and distributing them to all the participants. With this in mind, people have suggested using [financial data](https://eprint.iacr.org/2010/361), [national lotteries](https://en.bitcoin.it/wiki/Proof_of_burn), and more.
